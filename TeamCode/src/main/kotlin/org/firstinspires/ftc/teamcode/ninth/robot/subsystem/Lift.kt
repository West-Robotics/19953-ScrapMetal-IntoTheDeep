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

class Lift(hardwareMap: HardwareMap, val voltageMultiplier: Double = 1.0) {
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

    fun write() {
        left.write()
        right.write()
    }

    fun getHeight() = encoder.dist

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
                70.0,
                70.0,
                70.0
            ),
            mpTimer.seconds()
        )
        var effort = controlEffort(mpState.s, currentHeight, kp, 0.0)
//        if (preset != Preset.PULL_HANG) effort = effort.coerceAtLeast(-1.00)
        setEffort((effort + feedforward + kv * mpState.v + ka * mpState.a) * voltageMultiplier)

        return mpState
    }

    fun leftCurrent() = left.current

    fun rightCurrent() = right.current

    // TODO determine height to score specs
    enum class Preset(val height: Double) {
        BOTTOM(0.0),
        LOW(25.75 - 7.0),
        HIGH(43.0 - 7.0 - 0.25),
        RAISE_HANG(32.0),
        PULL_HANG(16.0),
        SPEC_INTAKE(12.0 - 7.0 + 1.5),
        SPEC_LOW(13.0 - 7.0 + 1.5 +  2.0),
        SPEC_LOW_SCORE(9.5 - 3.0),
        SPEC_HIGH(26.0 - 7.0 + 1.5 + 2.0),
        SPEC_HIGH_SCORE(22.5),
    }
}