package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.util.hardware.SMCRServo
import com.scrapmetal.util.hardware.SMServo

class Sampler(hardwareMap: HardwareMap) {
    private val extensionOne = SMServo(hardwareMap, "frontExt", SMServo.ModelPWM.AXON)
    private val extensionTwo = SMServo(hardwareMap, "backExt", SMServo.ModelPWM.AXON)

    // TODO: Make hardware private again
    val pivot = SMServo(hardwareMap, "pivot", SMServo.ModelPWM.AXON)
    val wrist = SMServo(hardwareMap, "wrist", SMServo.ModelPWM.AXON)
    private val intake = SMCRServo(hardwareMap, "intake", SMCRServo.ModelPWM.AXON, DcMotorSimple.Direction.FORWARD)

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
        SCORE(-0.20, 0.55, 0.00),
        SCORE_FRONT(-1.0,0.2,0.68),
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
    fun score_front() = setState(State.SCORE_FRONT)

    fun write() {
        extensionOne.write()
        extensionTwo.write()
    }
}