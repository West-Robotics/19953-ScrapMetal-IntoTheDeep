package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.hardware.SMMotor
import kotlin.math.abs
import kotlin.math.max

class Drivetrain(hardwareMap: HardwareMap) {
    private val frontLeft = SMMotor(hardwareMap, "frontLeft", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE)
    private val backLeft = SMMotor(hardwareMap, "backLeft", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE)
    private val backRight = SMMotor(hardwareMap, "backRight", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE)
    private val frontRight = SMMotor(hardwareMap, "frontRight", DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE)

    fun setVelocity(x: Double, y: Double, turn: Double) {
        val yModified: Double = y * 1.1
        val denominator = max(abs(x) + abs(y) + abs(turn), 1.0)

        frontLeft.effort = (x - yModified - turn) / denominator
        backLeft.effort = (x + yModified - turn) / denominator
        backRight.effort = (x - yModified + turn) / denominator
        frontRight.effort = (x + yModified + turn) / denominator
    }

    fun write() {
        frontLeft.write()
        backLeft.write()
        backRight.write()
        frontRight.write()
    }
}
