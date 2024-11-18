package org.firstinspires.ftc.teamcode.experimental.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.util.hardware.SMMotor

class Lift(hardwareMap: HardwareMap) {
    private val lift = SMMotor(
        hardwareMap,
        "lift",
        DcMotorSimple.Direction.FORWARD,
        DcMotor.ZeroPowerBehavior.BRAKE
    )

    fun setEffort(effort: Double) {
        lift.effort = effort
    }

    fun write() {
        lift.write()
    }
}