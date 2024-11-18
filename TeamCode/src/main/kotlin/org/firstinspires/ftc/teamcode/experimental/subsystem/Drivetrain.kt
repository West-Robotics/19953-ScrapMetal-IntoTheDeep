package org.firstinspires.ftc.teamcode.experimental.subsystem

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.control.Vector2d

class Drivetrain(hardwareMap: HardwareMap) {
    private val left = hardwareMap.dcMotor.get("left")
    private val right = hardwareMap.dcMotor.get("right")

    init {
        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.REVERSE
    }

    fun drive(v: Vector2d) {
        left.power = v.x - v.y
        right.power = v.x + v.y
    }
}