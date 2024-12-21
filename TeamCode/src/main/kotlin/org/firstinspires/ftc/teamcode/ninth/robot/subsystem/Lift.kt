package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.hardware.SMMotor
import com.scrapmetal.util.hardware.SMQuadrature
import org.firstinspires.ftc.teamcode.ninth.controlEffort
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap) {
    val feedforward = 0.1
    val kp = 1.5

    val cpr = 8192.0
    val spoolCircumference = 0.7874016 * PI

    private val left = SMMotor(hardwareMap, "leftLift", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT)
    private val right = SMMotor(hardwareMap, "rightLift", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.FLOAT)

    private val encoder = SMQuadrature(hardwareMap, "frontRight", spoolCircumference/cpr, 1.0/cpr, DcMotorSimple.Direction.REVERSE)

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

    fun runToPos(preset: Double, currentHeight: Double) {
        val effort = controlEffort(preset, currentHeight, kp, feedforward).coerceAtLeast(0.0)
        setEffort(effort)
    }

    fun leftCurrent() = left.current

    fun rightCurrent() = right.current
}