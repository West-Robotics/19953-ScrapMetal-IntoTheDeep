package org.firstinspires.ftc.teamcode.experimental.subsystem

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Lift(hardwareMap: HardwareMap) {
    private val lift = hardwareMap.dcMotor.get("lift")

    init {
        lift.direction = DcMotorSimple.Direction.FORWARD
    }

    fun setEffort(effort: Double) {
        lift.power = effort
    }
}