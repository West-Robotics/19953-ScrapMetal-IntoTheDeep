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
    val pitch = SMServo(hardwareMap, "pitch", 0.43, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    val roll = SMServo(hardwareMap, "roll", 0.00, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val intake = SMCRServo(hardwareMap, "intake", DcMotorSimple.Direction.REVERSE, SMCRServo.ModelPWM.AXON)

    // TODO: Determine values for specimen grabbing/ scoring, side sample grabbing (wrist/pivot)
    enum class State(val grabber: Double, val pivot: Double, val wrist: Double,  val linkage: Double) {
        EXTEND(0.00, 0.43, 0.0, 0.64),
        GRAB_SAMPLE(1.00, 0.12, 0.0, 0.64),
        GRAB_SAMPLE_SIDE(1.00, 0.00, 0.0, 0.68),
        GRAB_SPECIMEN (1.00, 0.12, 0.0, 0.68),
        SPIT(-1.00, 0.12, 0.0, 0.64),
        STOW(0.00, 0.43, 0.0, 0.03),
        HOLD(0.11, 0.43, 0.0, 0.03),
        PREPARE_TO_SCORE_SAMPLE (0.10, 0.55, 0.0,0.00),
        SCORE_SAMPLE(-0.20, 0.55, 0.0,0.00),
        PREPARE_TO_SCORE_SPECIMEN(-0.20, 0.55, 0.0,0.00),
        SCORE_SPECIMEN(-0.20, 0.55, 0.0,0.00),
        SCORE_FRONT(-1.0,0.2, 0.0,0.68),
    }

    // have continuous power (but less than intake) going during stow
    fun setState(state: State) {
        intake.effort = state.grabber
        pitch.position = state.pivot
        extensionOne.position = state.linkage
        extensionTwo.position = state.linkage
    }

    fun setExtensionAmount(extension: Double) {
        extensionOne.position = extension.coerceIn(0.0..0.68)
    }

    fun extend() = setState(State.EXTEND)
    fun grab_sample() = setState(State.GRAB_SAMPLE)
    fun grab_sample_side() = setState(State.GRAB_SAMPLE_SIDE)
    fun grab_specimen() = setState(State.GRAB_SPECIMEN)
    fun spit() = setState(State.SPIT)
    fun stow() = setState(State.STOW)
    fun hold() = setState(State.HOLD)
    fun score_sample() = setState(State.SCORE_SAMPLE)
    fun prepare_to_score_sample() = setState(State.PREPARE_TO_SCORE_SAMPLE)
    fun prepare_to_score_specimen() = setState(State.PREPARE_TO_SCORE_SPECIMEN)
    fun score_specimen() = setState(State.SCORE_SPECIMEN)
    fun score_front() = setState(State.SCORE_FRONT)

    fun write() {
        extensionOne.write()
        extensionTwo.write()
        roll.write()
        pitch.write()
        intake.write()
    }
}