package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Sampler(hardwareMap: HardwareMap) {
    private val extensionOne = hardwareMap.get(Servo::class.java, "extOne")
    private val extensionTwo = hardwareMap.get(Servo::class.java, "extTwo")

    private val pivot = hardwareMap.get(Servo::class.java, "pivot")
    private val intake = hardwareMap.get(Servo::class.java, "intake")

    init {
        extensionOne.setDirection(Servo.Direction.FORWARD)
        extensionTwo.setDirection(Servo.Direction.REVERSE)
    }

    enum class State(val grab: Double, val pivot: Double) {
        GRAB(0.0 , 0.0),
        STOW(0.0 , 0.0),
        OUTPUT(0.0 , 0.0),
    }

//    fun setState(state: State) {
//        intakeWheel.setPosition(state.grab)
//    }
}