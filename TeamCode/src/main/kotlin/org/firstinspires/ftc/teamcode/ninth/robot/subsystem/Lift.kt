package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.PI

class Lift(hardwareMap: HardwareMap) {
    private val left = hardwareMap.dcMotor.get("leftLift")
    private val right = hardwareMap.dcMotor.get("rightLift")
    // TODO: rename encoder to drive motor
    private val encoder = hardwareMap.dcMotor.get("encoder")

    init {
        left.setDirection(DcMotorSimple.Direction.FORWARD)
        right.setDirection(DcMotorSimple.Direction.REVERSE)

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    fun resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    }

    val cpr = 8192
    val spoolCircumference = 0.7874016 * PI

    fun inchesToTicks(inches: Double) = (inches * cpr/spoolCircumference).toInt()

    fun ticksToInches(ticks: Int) = ticks * spoolCircumference / cpr

    fun setPower(power: Double) {
        left.setPower(power)
        right.setPower(power)
    }

    fun getHeight() = ticksToInches(encoder.getCurrentPosition())

    fun runToPreset(preset: Double, currentHeight: Double, power: Double): Double {
        val speedForLoop = when {
            currentHeight < preset -> power
            currentHeight > preset -> -power
            else -> 0.0
        }
        return speedForLoop
    }

    fun controlEffort(preset: Double, currentHeight: Double, k: Double) = k*(currentHeight-preset)
}