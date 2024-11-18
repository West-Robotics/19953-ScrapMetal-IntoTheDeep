package org.firstinspires.ftc.teamcode.experimental.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Claw(hardwareMap: HardwareMap) {
    private val pinch = hardwareMap.servo.get("pinch")

    init {
        pinch.direction = Servo.Direction.REVERSE
    }

    fun setState(state: State) {
        pinch.position = state.pinch
    }

    enum class State(val pinch: Double) {
        OPEN(1.0),
        GRAB(0.0),
    }
}