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

class Lift(hardwareMap: HardwareMap, private val voltageMultiplier: Double = 1.0, val drivetrain: Drivetrain?, val auto: Boolean = false) {
    constructor(hardwareMap: HardwareMap, voltageMultiplier: Double = 1.0, auto: Boolean = false) : this(hardwareMap, voltageMultiplier, null, auto)
    val feedforward = 0.20
//    val kv = 0.023
    val kv = 0.0
    val ka = 0.000
    val kp = if (auto) 0.9 else 1.5
//    val kp = 0.0

    val cpr = 8192.0
    val spoolCircumference = 0.9842520 * PI

    private val left = SMMotor(hardwareMap, "leftLift", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT)
    private val right = SMMotor(hardwareMap, "rightLift", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT)
    private val encoder = SMQuadrature(hardwareMap, "frontRight", spoolCircumference/cpr, 1.0/cpr, DcMotorSimple.Direction.REVERSE)
    // TODO: make private
    val pto = SMServo(hardwareMap, "pto", PTO.STOW.position, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)

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
                accel = 4000.0,
                decel = if (mpStart < preset.height) 200.0 else if (auto) 80.0 else 120.0,
                vLimit = if (mpStart < preset.height) 40.0 else 70.0,
            ),
            mpTimer.seconds()
        )
        val effort = controlEffort(mpState.s, currentHeight, kp, 0.0)
        setEffort((effort + feedforward + kv * mpState.v + ka * mpState.a) * voltageMultiplier)

        if (debug != null) {
            debug.addData("desired height", mpState.s)
        }
    }

    fun updatePid(currentHeight: Double) {
        val effort = pControl(kp, Preset.PULL_CLIMB.height, currentHeight)
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
    fun pto2ENGAGE() {
        pto.position = PTO.ENGAGE.position
    }

    /**
     * USE IN FSM LOOP
     */
    fun pto3CLIMB(currentHeight: Double) {
        if (drivetrain != null) {
            val effort = pControl(kp, Preset.PULL_CLIMB.height, currentHeight)
            setEffort(effort)
            drivetrain.setWheels(effort, effort, 0.0, 0.0)
        }
    }

    enum class Preset(val height: Double) {
        BOTTOM         (00.00                        ),
        SIDE_SIN       (01.00                        ),
        SAMP_LOW       (25.75 - 7.0 + 1.0            ),
        SAMP_HIGH      (43.00 - 7.0 - 0.0            ),
        RAISE_CLIMB    (32.00                        ),
        PULL_CLIMB     (18.00                        ),
        SPEC_INTAKE    (12.00 - 7.0 + 1.5            ),
        SPEC_MID       (12.00 - 7.0 + 2.5            ),
        SPEC_LOW       (13.00 - 7.0 + 1.5 + 2.0      ),
        SPEC_LOW_SCORE (13.00 - 7.0 + 1.5 + 2.0 - 3.0),
        SPEC_HIGH      (26.00 - 7.0 + 1.5 + 2.0      ),
        SPEC_HIGH_SCORE(26.00 - 7.0 + 1.5 + 2.0 - 3.0),
    }

    enum class PTO(val position: Double) {
        STOW(0.2),
        ENGAGE(0.4),
    }
}