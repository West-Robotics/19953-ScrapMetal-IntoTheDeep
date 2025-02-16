package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.MPConstraints
import com.scrapmetal.util.control.MPState
import com.scrapmetal.util.control.motionProfile
import com.scrapmetal.util.control.pControl
import com.scrapmetal.util.hardware.SMMotor
import com.scrapmetal.util.hardware.SMQuadrature
import com.scrapmetal.util.hardware.SMServo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.ninth.controlEffort
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap, private val voltageMultiplier: Double = 1.0, val drivetrain: Drivetrain?, val auto: Boolean = false, val spec: Boolean = false) {
    constructor(hardwareMap: HardwareMap, voltageMultiplier: Double = 1.0, auto: Boolean = false, spec: Boolean = false) : this(hardwareMap, voltageMultiplier, null, auto, spec)
    val feedforward = 0.28
    val kv = 0.018
    val kaccel = 0.002
    val kdecelup = 0.0008
    val kdeceldown = if (!spec) 0.0007 else 0.0007
//    val kv = 0.0
//    val ka = 0.0
//    val kp = if (auto) 0.2 else 1.5
//    val kp = 0.8
    val kp = 0.3
    val kpSpecScore = if (!auto) 2.5 else 0.8
    val kpClimb = 0.5

    val cpr = 8192.0
    val spoolCircumference = 1.25984 * PI

    private val left = SMMotor(hardwareMap, "leftLift", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT)
    private val right = SMMotor(hardwareMap, "rightLift", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT)
    private val encoder = SMQuadrature(hardwareMap, "frontRight", spoolCircumference/cpr, 1.0/cpr, DcMotorSimple.Direction.FORWARD)
    // TODO: make private
    val pto = SMServo(hardwareMap, "pto", PTO.STOW.position, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)

    private var height = encoder.dist
    private var preset = Preset.BOTTOM
    private var mpStart = getHeight()
    private val mpTimer = ElapsedTime()
    private var ptoEngaged = false

    fun resetEncoder() {
        encoder.reset()
    }

    fun setEffort(power: Double) {
        left.effort = power
        right.effort = power
    }

    fun read() {
        height = encoder.dist
    }

    fun write() {
        left.write()
        right.write()
        pto.write()
    }

    fun getHeight() = height

    fun getEffort() = left.effort

    fun setPreset(preset: Preset) {
        this.preset = preset
        mpStart = getHeight()
        mpTimer.reset()
    }

    fun getPreset() = preset

    fun updateProfiled(currentHeight: Double) = updateProfiled(currentHeight, debug = null)

    fun updateProfiled(currentHeight: Double, debug: Telemetry?) {
        val mpState = motionProfile(
            MPConstraints(
                start = mpStart,
                end = preset.height,
                accel = 800.0,
//                decel = if (mpStart < preset.height) 250.0 else if (auto) 50.0 else 250.0,
//                vLimit = if (mpStart < preset.height) 40.0 else if (auto) 30.0 else 70.0,
                decel = if (mpStart < preset.height) 250.0 else 250.0,
                vLimit = if (mpStart < preset.height) 40.0 else (if (!spec) 70.0 else 40.0),
//                vLimit = if (mpStart < preset.height) 50.0 else 70.0,
            ),
            mpTimer.seconds()
        )
        val effort = controlEffort(mpState.s, currentHeight, kp, 0.0)
        val component1 = (if (currentHeight < 1.0 && auto) effort * 2.0 else effort)
        val component2 = (if (currentHeight < 0.75) 0.0 else feedforward)
        val component3 = (if (mpStart < preset.height) kv else 0.010) * mpState.v
        val component4 = (when {
            (mpStart <= preset.height && mpState.a > 0) || (mpStart > preset.height && mpState.a < 0) -> kaccel
            (mpStart <= preset.height && mpState.a < 0) -> kdecelup
            (mpStart > preset.height && mpState.a > 0) -> kdeceldown
            else -> 0.0
        }) * mpState.a
        val totalEffort = (component1 + component2 + component3 + component4) * voltageMultiplier
        setEffort(totalEffort)

        if (debug != null) {
            debug.addData("desired height", mpState.s)
            debug.addData("desired velo", mpState.v)
            debug.addData("desired accel", mpState.a)
            debug.addData("component1", component1)
            debug.addData("component2", component2)
            debug.addData("component3", component3)
            debug.addData("component4", component4)
            debug.addData("total lift effort", totalEffort)
        }
    }

    fun updatePid(currentHeight: Double) {
        val effort = pControl(kpSpecScore, getPreset().height, currentHeight)
        setEffort((effort + feedforward) * voltageMultiplier)
    }

    fun leftCurrent() = left.current

    fun rightCurrent() = right.current

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
            val effort = pControl(kpClimb, Preset.PULL_CLIMB.height, currentHeight)
            setEffort(effort)
            drivetrain.setWheels(effort, 0.0, 0.0, effort)
        }
    }

    enum class Preset(val height: Double) {
        BOTTOM         (00.00                        ),
        SIDE_SIN       (01.00                        ),
        SAMP_LOW       (25.75 - 7.0 + 1.0            ),
        INIT_POST_AUTO (25.75 - 7.0 + 0.0            ),
        SAMP_HIGH      (43.00 - 7.0 + 0.7            ),
        RAISE_CLIMB    (32.00                        ),
        PULL_CLIMB     (20.00                        ),
        SPEC_INTAKE    (12.00 - 7.0 + 0.5            ),
        SPEC_MID       (12.00 - 7.0 + 4.5            ),
        SPEC_LOW       (13.00 - 7.0 + 1.5 + 2.0      ),
        SPEC_LOW_SCORE (13.00 - 7.0 + 1.5 + 2.0 - 1.5),
        SPEC_HIGH      (26.00 - 7.0 + 1.5 + 3.5      ),
        SPEC_HIGH_SCORE(26.00 - 7.0 + 1.5 + 3.5 + 2.0),
    }

    enum class PTO(val position: Double) {
        STOW(0.68),
//        ENGAGE(0.71),
        ENGAGE(0.89),
    }
}