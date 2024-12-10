package org.firstinspires.ftc.teamcode.experimental.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.util.architecture.Subsystem

class Claw(hardwareMap: HardwareMap) : Subsystem {
    private val pinch = hardwareMap.servo.get("claw")
    private var pinchPos = State.CLOSE.pinch

    init {
        pinch.direction = Servo.Direction.FORWARD
    }

    fun setState(state: State) {
        pinchPos = state.pinch
    }

    enum class State(val pinch: Double) {
        OPEN(0.4),
        CLOSE(0.0),
    }

    override fun write() {
        pinch.position = pinchPos
    }
}