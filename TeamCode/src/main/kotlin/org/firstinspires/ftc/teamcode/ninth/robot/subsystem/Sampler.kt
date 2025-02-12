package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.MPConstraints
import com.scrapmetal.util.control.MPState
import com.scrapmetal.util.control.motionProfile
import com.scrapmetal.util.hardware.SMCRServo
import com.scrapmetal.util.hardware.SMServo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.ninth.controlEffort

class Sampler(hardwareMap: HardwareMap) {
    private val extensionOne = SMServo(hardwareMap, "frontExt", 0.00, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val extensionTwo = SMServo(hardwareMap, "backExt", 0.00, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    // TODO: Make hardware private again
    private val pitch = SMServo(hardwareMap, "pitch", 0.43, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val roll = SMServo(hardwareMap, "roll", 0.00, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val intake = SMCRServo(hardwareMap, "intake", DcMotorSimple.Direction.REVERSE, SMCRServo.ModelPWM.AXON)

    private var mpStart = State.STOW.pitch
    private var mpEnd = State.STOW.pitch
    private val mpTimer = ElapsedTime()

    enum class State(val grabber: Double, val pitch: Double, val roll: Double, val linkage: Double) {
        EXTEND                      ( 0.00, 0.43, 0.51, 0.64),
        GRAB_SAMPLE                 ( 1.00, 0.13, 0.51, 0.64),
//        GRAB_SAMPLE_LEFT_SIDE       ( 1.00, 0.09, 0.25, 0.64),
//        GRAB_SAMPLE_RIGHT_SIDE      ( 1.00, 0.09, 0.75, 0.64),
        GRAB_SAMPLE_LEFT_SIDE       ( 1.00, 0.125, 0.17, 0.64),
        GRAB_SAMPLE_RIGHT_SIDE      ( 1.00, 0.12, 0.85, 0.64),
        SPIT                        (-1.00, 0.12, 0.51, 0.64),
        STOW                        ( 0.00, 0.43, 0.51, 0.03),
        HOLD                        ( 0.20, 0.43, 0.51, 0.03),
        GRAB_SPECIMEN               ( 1.00, 0.20, 0.75, 0.64),
        LIFT_SPECIMEN               ( 0.20, 0.43, 0.75, 0.03),
        HOLD_SPECIMEN               ( 0.20, 0.43, 0.25, 0.03),
        PREPARE_TO_SCORE_SAMPLE     ( 0.15, 0.59, 0.51, 0.03),
        SCORE_SAMPLE                (-0.20, 0.59, 0.51, 0.03),
        PREPARE_TO_SCORE_SPECIMEN   ( 0.20, 0.68, 0.25, 0.03),
        SCORE_SPECIMEN              (-1.00, 0.68, 0.25, 0.03),
        SCORE_FRONT                 (-1.00, 0.20, 0.51, 0.64),
        SWEEP                       ( 0.00, 0.08, 0.51, 0.64),
    }

    fun setState(state: State) {
        intake.effort = state.grabber
        pitch.position = state.pitch
        roll.position = state.roll
        extensionOne.position = state.linkage
        extensionTwo.position = state.linkage
    }

    /**
     * Normal [setState] but with a motion profile on pitch
     */
    fun setStateProfiled(state: State) {
        intake.effort = state.grabber
        roll.position = state.roll
        mpStart = pitch.position
        mpEnd = state.pitch
        mpTimer.reset()
        extensionOne.position = state.linkage
        extensionTwo.position = state.linkage
    }

    fun updateProfiled() {
        pitch.position = motionProfile(
            MPConstraints(
                start = mpStart,
                end = mpEnd,
                accel = 8.0,
                decel = 4.0,
                vLimit = 2.0,
            ),
            mpTimer.seconds()
        ).s
    }

    fun setExtensionAmount(extension: Double) {
        extensionOne.position = extension.coerceIn(0.0..0.68)
    }

    fun extend() = setState(State.EXTEND)
    fun grab_sample() = setState(State.GRAB_SAMPLE)
    fun grab_sample_left_side() = setState(State.GRAB_SAMPLE_LEFT_SIDE)
    fun grab_sample_right_side() = setState(State.GRAB_SAMPLE_RIGHT_SIDE)
    fun spit() = setState(State.SPIT)
    fun stow() = setState(State.STOW)
    fun hold() = setState(State.HOLD)
    fun grab_specimen() = setState(State.GRAB_SPECIMEN)
    fun lift_specimen() = setState(State.LIFT_SPECIMEN)
    fun hold_specimen() = setState(State.HOLD_SPECIMEN)
    fun score_sample() = setState(State.SCORE_SAMPLE)
    fun prepare_to_score_sample() = setState(State.PREPARE_TO_SCORE_SAMPLE)
    fun prepare_to_score_specimen() = setStateProfiled(State.PREPARE_TO_SCORE_SPECIMEN)
    fun score_specimen() = setState(State.SCORE_SPECIMEN)
    fun score_front() = setState(State.SCORE_FRONT)
    fun sweep() = setState(State.SWEEP)

    fun write() {
        extensionOne.write()
        extensionTwo.write()
        roll.write()
        pitch.write()
        intake.write()
    }
}