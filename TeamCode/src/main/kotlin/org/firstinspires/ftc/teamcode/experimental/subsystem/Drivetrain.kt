package org.firstinspires.ftc.teamcode.experimental.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.hardware.SMMotor

class Drivetrain(hardwareMap: HardwareMap) {
    private val left = SMMotor(hardwareMap, "left", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT)
    private val right = SMMotor(hardwareMap, "right", DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT)

    fun drive(v: Vector2d) {
        left.effort = v.x - v.y
        right.effort = v.x + v.y
    }

    fun write() {
        left.write()
        right.write()
    }
}