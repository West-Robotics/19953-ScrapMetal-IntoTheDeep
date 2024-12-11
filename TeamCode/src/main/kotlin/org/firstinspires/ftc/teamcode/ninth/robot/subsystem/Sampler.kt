package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class Sampler(hardwareMap: HardwareMap) {
    private val extensionOne = hardwareMap.get(Servo::class.java, "frontExt")
    private val extensionTwo = hardwareMap.get(Servo::class.java, "backExt")

    // TODO: Make hardware private again
    val pivot = hardwareMap.get(Servo::class.java, "pivot")
    private val intake = hardwareMap.get(CRServo::class.java, "intake")

    init {
        pivot.setDirection(Servo.Direction.FORWARD)
        intake.setDirection(DcMotorSimple.Direction.FORWARD)
        extensionOne.setDirection(Servo.Direction.REVERSE)
        extensionTwo.setDirection(Servo.Direction.REVERSE)
    }

    enum class State(val grabber: Double, val pivot: Double, val linkage: Double) {
        EXTEND(0.00, 0.43, 0.68),
        GRAB(1.00, 0.03, 0.68),
        SPIT(-1.00, 0.03, 0.68),
        STOW(0.00, 0.43, 0.00),
//        RETRACT(0.0, 0.43, 0.0),
        HOLD(0.10, 0.43, 0.00),
        SCORE(-0.30, 0.5, 0.00),
    }

    // have continuous power (but less than intake) going during stow
    fun setState(state: State) {
        intake.setPower(state.grabber)
        pivot.setPosition(state.pivot)
        extensionOne.setPosition(state.linkage)
        extensionTwo.setPosition(state.linkage)
    }

    fun setExtensionAmount(extension: Double) {
        extensionOne.setPosition(extension.coerceIn(0.0..0.68))
    }

    fun extend() = setState(State.EXTEND)
    fun grab() = setState(State.GRAB)
    fun spit() = setState(State.SPIT)
    fun stow() = setState(State.STOW)
//    fun retract() = setState(State.RETRACT)
    fun hold() = setState(State.HOLD)
    fun score() = setState(State.SCORE)

    // TODO: Add time-based relationships between actions
}