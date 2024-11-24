package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.hardware.SMMotor
import com.scrapmetal.util.hardware.SMQuadrature
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap) {
    val cpr = 8192.0
    val spoolCircumference = 0.7874016 * PI

    private val left = SMMotor(hardwareMap, "leftLift", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE)
    private val right = SMMotor(hardwareMap, "rightLift", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE)

    private val encoder = SMQuadrature(hardwareMap, "frontRight", spoolCircumference/cpr, 1.0/cpr, DcMotorSimple.Direction.REVERSE)

    fun resetEncoder() {
        encoder.reset()
    }

    fun inchesToTicks(inches: Double) = (inches * cpr/spoolCircumference).toInt()

    fun ticksToInches(ticks: Int) = ticks * spoolCircumference / cpr

    fun setPower(power: Double) {
        left.effort = power
        right.effort = power
    }

    fun writeLiftEffort() {
        left.write()
        right.write()
    }

    fun getHeight() = encoder.dist

    fun runToPreset(preset: Double, currentHeight: Double, power: Double): Double {
        val speedForLoop = when {
            currentHeight < preset -> power
            currentHeight > preset -> -power
            else -> 0.0
        }
        return speedForLoop
    }

    fun controlEffort(preset: Double, currentHeight: Double, k: Double, f: Double) = k*(currentHeight-preset) + f
}