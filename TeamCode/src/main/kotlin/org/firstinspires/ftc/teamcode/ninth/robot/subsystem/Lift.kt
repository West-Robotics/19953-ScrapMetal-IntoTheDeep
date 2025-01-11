package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.MPConstraints
import com.scrapmetal.util.control.motionProfile
import com.scrapmetal.util.hardware.SMMotor
import com.scrapmetal.util.hardware.SMQuadrature
import org.firstinspires.ftc.teamcode.ninth.controlEffort
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap) {
    val feedforward = 0.1
    val kp = 2.5

    val cpr = 8192.0
    val spoolCircumference = 0.7874016 * PI

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

    fun updateProfiled(currentHeight: Double) {
        val reference = motionProfile(
            MPConstraints(
                mpStart,
                preset.height,
                1000.0,
                100.0,
                100.0
            ),
            mpTimer.seconds()
        ).s
        val effort = controlEffort(reference, currentHeight, kp, feedforward).coerceAtLeast(0.0)
        setEffort(effort)
    }

    fun leftCurrent() = left.current

    fun rightCurrent() = right.current

    enum class Preset(val height: Double) {
        BOTTOM(0.0),
        LOW(25.75 - 7.0),
        HIGH(43.0 - 7.0),
        RAISE_HANG(43.0 - 7.0),
        PULL_HANG(43.0 - 13.0),
    }
}