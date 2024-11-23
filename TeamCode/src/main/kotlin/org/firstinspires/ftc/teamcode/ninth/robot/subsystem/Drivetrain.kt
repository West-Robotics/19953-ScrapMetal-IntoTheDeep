package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.max

class Drivetrain(hardwareMap: HardwareMap) {
    private val frontLeft = hardwareMap.get(DcMotor::class.java, "frontLeft")
    private val backLeft = hardwareMap.get(DcMotor::class.java, "backLeft")
    private val backRight = hardwareMap.get(DcMotor::class.java, "backRight")
    private val frontRight = hardwareMap.get(DcMotor::class.java, "frontRight")


    init {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD)
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE)
        backRight.setDirection(DcMotorSimple.Direction.FORWARD)
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE)

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    fun setVelocity(x: Double, y: Double, turn: Double) {
        val yModified: Double = y * 1.1
        val denominator = max(abs(x) + abs(y) + abs(turn), 1.0)

        frontLeft.setPower((x - yModified - turn) / denominator)
        backLeft.setPower((x + yModified - turn) / denominator)
        backRight.setPower((x - yModified + turn) / denominator)
        frontRight.setPower((x + yModified + turn) / denominator)
    }
}
