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

    enum class State(val grabber: Double, val pitch: Double, val roll: Double, val linkage: Double) {
        EXTEND                      ( 0.00, 0.43, 0.51, 0.64),
        GRAB_SAMPLE                 ( 1.00, 0.12, 0.51, 0.64),
        GRAB_SAMPLE_LEFT_SIDE       ( 1.00, 0.06, 0.25, 0.64),
        GRAB_SAMPLE_RIGHT_SIDE      ( 1.00, 0.06, 0.76, 0.64),
        GRAB_SPECIMEN               ( 1.00, 0.20, 0.75, 0.64),
        SPIT                        (-1.00, 0.12, 0.51, 0.64),
        STOW                        ( 0.00, 0.43, 0.51, 0.03),
        HOLD                        ( 0.11, 0.43, 0.51, 0.03),
        HOLD_SPECIMEN               ( 0.20, 0.43, 0.25, 0.03),
        PREPARE_TO_SCORE_SAMPLE     ( 0.11, 0.62, 0.51, 0.03),
        SCORE_SAMPLE                (-0.20, 0.62, 0.51, 0.03),
        PREPARE_TO_SCORE_SPECIMEN   ( 0.20, 0.68, 0.25, 0.03),
        SCORE_SPECIMEN              (-0.20, 0.68, 0.25, 0.03),
        SCORE_FRONT                 (-1.00, 0.20, 0.51, 0.64),
    }

    fun setState(state: State) {
        intake.effort = state.grabber
        pitch.position = state.pitch
        roll.position = state.roll
        extensionOne.position = state.linkage
        extensionTwo.position = state.linkage
    }

    fun setExtensionAmount(extension: Double) {
        extensionOne.position = extension.coerceIn(0.0..0.68)
    }

    fun extend() = setState(State.EXTEND)
    fun grab_sample() = setState(State.GRAB_SAMPLE)
    fun grab_sample_left_side() = setState(State.GRAB_SAMPLE_LEFT_SIDE)
    fun grab_sample_right_side() = setState(State.GRAB_SAMPLE_RIGHT_SIDE)
    fun grab_specimen() = setState(State.GRAB_SPECIMEN)
    fun spit() = setState(State.SPIT)
    fun stow() = setState(State.STOW)
    fun hold() = setState(State.HOLD)
    fun hold_specimen() = setState(State.HOLD_SPECIMEN)
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