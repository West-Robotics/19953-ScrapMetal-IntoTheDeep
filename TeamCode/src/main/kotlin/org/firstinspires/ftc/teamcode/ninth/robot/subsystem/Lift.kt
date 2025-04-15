package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.MPConstraints
import com.scrapmetal.util.control.feedforward
import com.scrapmetal.util.control.motionProfile
import com.scrapmetal.util.control.pControl
import com.scrapmetal.util.hardware.SMMotor
import com.scrapmetal.util.hardware.SMQuadrature
import com.scrapmetal.util.hardware.SMServo
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap, private val voltageMultiplier: Double = 1.0, val drivetrain: Drivetrain?, val auto: Boolean = false) {
    constructor(hardwareMap: HardwareMap, voltageMultiplier: Double = 1.0, auto: Boolean = false) : this(hardwareMap, voltageMultiplier, null, auto)
    val ff = 0.28
    val kVUp = 0.052
    val kVDown = 0.014
    val kAccel = 0.002
    val kDecelUp = 0.0008
    val kDecelDown = 0.0007
//    val kv = 0.0
//    val ka = 0.0
//    val kp = if (auto) 0.2 else 1.5
//    val kp = 0.8
    val kP = 0.7
    val kD = 0.02
    val kPSpecScore = if (!auto) 2.5 else 0.8
    val kPClimb = 0.5

    val cpr = 8192.0
    val spoolCircumference = 1.25984 * PI

    private val left = SMMotor(hardwareMap, "leftLift", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT)
    private val right = SMMotor(hardwareMap, "rightLift", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT)
    private val encoder = SMQuadrature(
        hardwareMap,
        "frontRight",
        distPerTick = spoolCircumference/cpr,
        revsPerTick = 1.0/cpr,
        DcMotorSimple.Direction.REVERSE,
        beta = 0.3,
    )
    // TODO: make private
    val pto = SMServo(hardwareMap, "pto", PTO.STOW.position, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)

    var height = encoder.dist
        private set
    var velo = encoder.linearV
        private set
    var effort = 0.0
        set(value) {
            field = value
            left.effort = value
            right.effort = value
        }
    var preset = Preset.BOTTOM
        set(value) {
            field = value
            mpStart = height
            mpTimer.reset()
        }
    private var mpStart = height
    private val mpTimer = ElapsedTime()
    private var ptoEngaged = false

    val leftCurrent
        get() = left.current
    val rightCurrent
        get() = right.current

    fun resetEncoder() {
        encoder.reset()
    }

    fun trueResetEncoder() {
        encoder.trueReset()
    }

    fun read() {
        encoder.update()
        height = encoder.dist
        velo = encoder.linearV
    }

    fun write() {
        left.write()
        right.write()
        pto.write()
    }

    fun updateProfiled(currentHeight: Double) = updateProfiled(currentHeight, debug = null)

    fun updateProfiled(currentHeight: Double, debug: Telemetry?) {
        val mpState = motionProfile(
            MPConstraints(
                start = mpStart,
                end = preset.height,
                accel = 800.0,
//                decel = if (mpStart < preset.height) 250.0 else if (auto) 50.0 else 250.0,
//                vLimit = if (mpStart < preset.height) 40.0 else if (auto) 30.0 else 70.0,
                decel = if (mpStart < preset.height) 250.0 else 200.0,
                vLimit = if (mpStart < preset.height) 35.0 else 70.0,
//                vLimit = if (mpStart < preset.height) 50.0 else 70.0,
            ),
            mpTimer.seconds()
        )
        val feedback = pControl(kP, mpState.s, currentHeight) + pControl(kD, mpState.v, velo)
        val ffG = if (currentHeight < 1.0) 0.0 else ff
        val ffV = (if (mpStart < preset.height) kVUp else kVDown) * mpState.v
        val ffA = (when {
            (mpStart <= preset.height && mpState.a > 0) || (mpStart > preset.height && mpState.a < 0) -> kAccel
            (mpStart <= preset.height && mpState.a < 0) -> kDecelUp
            (mpStart > preset.height && mpState.a > 0) -> kDecelDown
            else -> 0.0
        }) * mpState.a
        val totalEffort = (ffG + ffV + feedback) * voltageMultiplier
        // val totalEffort = (feedback + ffG) * voltageMultiplier
        effort = totalEffort

        if (debug != null) {
            debug.addData("desired height", mpState.s)
            debug.addData("desired velo", mpState.v)
            debug.addData("desired accel", mpState.a)
            debug.addData("height", height)
            debug.addData("velo", velo)
            debug.addData("feedback", feedback)
            debug.addData("ffG", ffG)
            debug.addData("ffV", ffV)
            debug.addData("ffA", ffA)
            debug.addData("total lift effort", totalEffort)
        }
    }

    fun updatePid(currentHeight: Double) {
        effort = (pControl(kPSpecScore, preset.height, currentHeight) + ff) * voltageMultiplier
    }

    /**
     * USE IN FSM LOOP
     */
    fun pto1FREEZE() {
        drivetrain?.setEffort(0.0, 0.0, 0.0)
    }

    /**
     * USE IN FSM LOOP
     */
    fun pto1ENGAGE() {
        pto.position = PTO.ENGAGE.position
    }

    /**
     * USE IN FSM LOOP
     */
    fun pto2CLIMB(currentHeight: Double) {
        if (drivetrain != null) {
            effort = pControl(kPClimb, Preset.PULL_CLIMB.height, currentHeight)
            drivetrain.setWheels(effort, 0.0, 0.0, effort)
        }
    }

    fun pto2MANUALCLIMB(input: Double) {
        if (drivetrain != null) {
            effort = input
            drivetrain.setWheels(input, 0.0, 0.0, input)
        }
    }

    enum class Preset(val height: Double) {
        BOTTOM         (00.00                        ),
        SIDE_SIN       (01.00                        ),
        // AUTO_SPIKE     (                             ),
        SAMP_LOW       (25.75 - 7.0 + 1.0-0.0        ), // -4
        INIT_POST_AUTO (25.75 - 7.0 + 0.0            ),
        SAMP_HIGH      (43.00 - 7.0 + 0.7-3.0        ),
        RAISE_CLIMB    (32.00                        ),
        PULL_CLIMB     (20.00                        ),
        SPEC_HIGH      (26.00 - 7.0 + 1.5 - 6.0      ),
    }

    enum class PTO(val position: Double) {
        STOW(0.68),
//        ENGAGE(0.71),
        ENGAGE(0.89),
    }
}