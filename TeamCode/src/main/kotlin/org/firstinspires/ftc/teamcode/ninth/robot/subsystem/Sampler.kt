package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.util.hardware.SMCRServo
import com.scrapmetal.util.hardware.SMServo

class Sampler(hardwareMap: HardwareMap) {
    private val extensionOne = SMServo(hardwareMap, "frontExt", 0.00, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val extensionTwo = SMServo(hardwareMap, "backExt", 0.00, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)

    // TODO: Make hardware private again
    private val pivot = SMServo(hardwareMap, "pivot", 0.43, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val wrist = SMServo(hardwareMap, "wrist", 0.00, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val intake = SMCRServo(hardwareMap, "intake", DcMotorSimple.Direction.REVERSE, SMCRServo.ModelPWM.AXON)

//    // intake
//    val WAITING = 0.0
//    val HOLDING = 0.1
//    val GRABBING = 1.0
//    val SPITTING = -1.0
//
//    // linkage
//    val STOWING = 0.0
//    val EXTENDING = 0.68
//
//    // pivot
//    val INTAKING = 0.03
//    val KEEPING = 0.43
//    val SCORING = 0.55
//
//    //wrist
//    val SPECIMINING = 0.0
//    val SAMPLING = 0.0
//    val W_HOLDING = 0.0

    enum class State(val grabber: Double, val pivot: Double, val wrist: Double,  val linkage: Double) {
        EXTEND(0.00, 0.43, 0.0, 0.64),
        GRAB_SAMPLE(1.00, 0.12, 0.0, 0.64),
        GRAB_SPECIMEN (1.00, 0.12, 0.0, 0.00),
        SPIT(-1.00, 0.12, 0.0, 0.64),
        STOW(0.00, 0.43, 0.0, 0.03),
//        RETRACT(0.0, 0.43, 0.0),
        HOLD(0.10, 0.43, 0.0, 0.03),
        SCORE_SAMPLE(-0.17, 0.55, 0.0, 0.03),
        SCORE_SPECIMEN(-0.20, 0.55, 0.0, 0.03),
        SCORE_FRONT(-1.0, 0.2, 0.0, 0.64),
    }

    // have continuous power (but less than intake) going during stow
    fun setState(state: State) {
        intake.effort = state.grabber
        pivot.position = state.pivot
        extensionOne.position = state.linkage
        extensionTwo.position = state.linkage
    }

    fun setExtensionAmount(extension: Double) {
        extensionOne.position = extension.coerceIn(0.0..0.68)
    }

    fun extend() = setState(State.EXTEND)
    fun grab_sample() = setState(State.GRAB_SAMPLE)
    fun grab_specimen() = setState(State.GRAB_SPECIMEN)
    fun spit() = setState(State.SPIT)
    fun stow() = setState(State.STOW)
//    fun retract() = setState(State.RETRACT)
    fun hold() = setState(State.HOLD)
    fun score_sample() = setState(State.SCORE_SAMPLE)
    fun score_specimen() = setState(State.SCORE_SPECIMEN)
    fun score_front() = setState(State.SCORE_FRONT)

    fun write() {
        extensionOne.write()
        extensionTwo.write()
        wrist.write()
        pivot.write()
        intake.write()
    }
}