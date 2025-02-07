package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.MPConstraints
import com.scrapmetal.util.control.MPState
import com.scrapmetal.util.control.motionProfile
import com.scrapmetal.util.hardware.SMMotor
import com.scrapmetal.util.hardware.SMQuadrature
import org.firstinspires.ftc.teamcode.ninth.controlEffort
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap, private val voltageMultiplier: Double = 1.0) {
    val feedforward = 0.20
//    val kv = 0.023
    val kv = 0.0
    val ka = 0.000
    val kp = 1.5
//    val kp = 0.0

    val cpr = 8192.0
    val spoolCircumference = 0.9842520 * PI

    private val left = SMMotor(hardwareMap, "leftLift", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT)
    private val right = SMMotor(hardwareMap, "rightLift", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT)
    private val encoder = SMQuadrature(hardwareMap, "frontRight", spoolCircumference/cpr, 1.0/cpr, DcMotorSimple.Direction.REVERSE)

    private var height = encoder.dist
    private var preset = Preset.BOTTOM
    private var mpStart = getHeight()
    private val mpTimer = ElapsedTime()

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

    fun updateProfiled(currentHeight: Double): MPState {
        val mpState = motionProfile(
            MPConstraints(
                mpStart,
                preset.height,
                700.0,
                70.0,
                100.0
            ),
            mpTimer.seconds()
        )
        val effort = controlEffort(mpState.s, currentHeight, kp, 0.0)
        setEffort((effort + feedforward + kv * mpState.v + ka * mpState.a) * voltageMultiplier)

        return mpState
    }

    fun updatePid(currentHeight: Double) {
        val effort = controlEffort(preset.height, currentHeight, kp, 0.0)
        setEffort((effort + feedforward) * voltageMultiplier)
    }

    fun leftCurrent() = left.current

    fun rightCurrent() = right.current

    enum class Preset(val height: Double) {
        BOTTOM         (00.00                  ),
        SIDE_SIN       (01.00                  ),
        SAMP_LOW       (25.75 - 7.0            ),
        SAMP_HIGH      (43.00 - 7.0 - 0.25     ),
        RAISE_HANG     (32.00                  ),
        PULL_HANG      (14.00                  ),
        SPEC_INTAKE    (12.00 - 7.0 + 1.5      ),
        SPEC_LOW       (13.00 - 7.0 + 1.5 + 2.0),
        SPEC_LOW_SCORE (09.50 - 3.0            ),
        SPEC_HIGH      (26.00 - 7.0 + 1.5 + 2.0),
        SPEC_HIGH_SCORE(19.50                  ),
    }
}