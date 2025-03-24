package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.MPConstraints
import com.scrapmetal.util.control.motionProfile
import com.scrapmetal.util.hardware.SMCRServo
import com.scrapmetal.util.hardware.SMServo

class Sampler(hardwareMap: HardwareMap) {
    private val extensionOne = SMServo(hardwareMap, "frontExt", 0.60, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val extensionTwo = SMServo(hardwareMap, "backExt", 0.60, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    // TODO: Make hardware private again
    private val pitch = SMServo(hardwareMap, "pitch", 0.34, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val subPitch = SMServo(hardwareMap, "subPitch", 0.11, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val roll = SMServo(hardwareMap, "roll", 0.10, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val claw = SMServo(hardwareMap, "pinch", 0.00, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)

    private var mpStart = State.STOW.pitch
    private var mpEnd = State.STOW.pitch
    private val mpTimer = ElapsedTime()
    private var pitchOffset = 0.0

    // TODO: rename grab states to something more discriptive 
    enum class State(val claw: Double, val pitch: Double, val subPitch: Double, val roll: Double, val linkage: Double) {
        EXTEND                      ( 0.00, 0.34, 0.11,0.10, 0.60),
        GRAB_SAMPLE                 ( 0.00, 0.54, 0.37,0.36, 0.60),
        GRAB_SAMPLE_LEFT_SIDE       ( 0.00, 0.54, 0.37,0.02, 0.60),
        GRAB_SAMPLE_RIGHT_SIDE      ( 0.00, 0.54, 0.37,0.21, 0.60),
        GRAB_SAMPLE_SIDE            ( 0.00, 0.54, 0.37,0.10, 0.60),
        SPIT                        ( 0.00, 0.54, 0.37,0.36, 0.60),
        STOW                        ( 0.00, 0.34, 0.00,0.10, 0.03),
        HOLD                        ( 0.27, 0.34, 0.00,0.10, 0.03),
        HOLD_SAMPLE                 ( 0.27, 0.34, 0.00,0.10, 0.03),
        PREPARE_TO_SCORE_SAMPLE     ( 0.27, 0.59, 0.00,0.10, 0.03),
        SCORE_SAMPLE                ( 0.00, 0.59, 0.00,0.10, 0.03),
        GRAB_SPECIMEN               ( 0.00, 0.12, 0.00,0.10, 0.36),
        LIFT_SPECIMEN               ( 0.27, 0.22, 0.00,0.10, 0.36),
        HOLD_SPECIMEN               ( 0.27, 0.22, 0.00,0.10, 0.36),
        DIP_SPECIMEN                ( 0.00, 0.08, 0.00,0.10, 0.36),
        RETRACT_SPECIMEN            ( 0.27, 0.10, 0.00,0.10, 0.28),
        SCORE_SPECIMEN              ( 0.27, 0.24, 0.00,0.10, 0.28),
        RELEASE_SPECIMEN            ( 0.00, 0.24, 0.00,0.10, 0.28),
        SCORE_FRONT                 ( 0.00, 0.20, 0.00,0.10, 0.60),
        SWEEP                       ( 0.00, 0.08, 0.00,0.10, 0.60),
        PREPARE_TO_SCORE_SPECIMEN   ( 0.27, 0.45, 0.00,0.10, 0.40),
        SPEC_PRELOAD                ( 0.00, 0.40, 0.00,0.10, 0.36),
    }

    /**
     * Normal [setState] but with a motion profile on pitch
     */
    fun setState(state: State) {
        claw.position = state.claw
        roll.position = state.roll
        subPitch.position = state.subPitch
        mpStart = pitch.position
        mpEnd = state.pitch
        mpTimer.reset()
        extensionOne.position = state.linkage
        extensionTwo.position = state.linkage
    }

    fun updateProfiled(retracting: Boolean = false) {
        pitch.position = motionProfile(
            if (retracting) {
                MPConstraints(
                    start = mpStart,
                    end = mpEnd,
                    accel = 4.0,
                    decel = 4.0,
                    vLimit = 2.0,
                )
            } else {
                MPConstraints(
                    start = mpStart,
                    end = mpEnd,
                    accel = 10.0,
                    decel = 4.0,
                    vLimit = 8.0,
                )
            },
            mpTimer.seconds()
        ).s + pitchOffset
    }

    // TODO: maybe increase servo caching precision
    fun incrementPitch() { pitchOffset += 0.01 }

    fun decrementPitch() { pitchOffset -= 0.01 }

    fun extend() = setState(State.EXTEND)
    fun grab_sample() = setState(State.GRAB_SAMPLE)
    fun grab_sample_left_side() = setState(State.GRAB_SAMPLE_LEFT_SIDE)
    fun grab_sample_right_side() = setState(State.GRAB_SAMPLE_RIGHT_SIDE)
    fun grab_sample_side() = setState(State.GRAB_SAMPLE_SIDE)
    fun spit() = setState(State.SPIT)
    fun stow() = setState(State.STOW)
    fun hold() = setState(State.HOLD)
    fun hold_sampele() = setState(State.HOLD_SAMPLE)
    fun grab_specimen() = setState(State.GRAB_SPECIMEN)
    fun lift_specimen() = setState(State.LIFT_SPECIMEN)
    fun hold_specimen() = setState(State.HOLD_SPECIMEN)
    fun score_sample() = setState(State.SCORE_SAMPLE)
    fun prepare_to_score_sample() = setState(State.PREPARE_TO_SCORE_SAMPLE)
    fun prepare_to_score_specimen() = setState(State.PREPARE_TO_SCORE_SPECIMEN)
    fun dip_specimen() = setState(State.DIP_SPECIMEN)
    fun dip_specimen_fast() = setState(State.DIP_SPECIMEN)
    fun retract_specimen() = setState(State.RETRACT_SPECIMEN)
    fun score_specimen() = setState(State.SCORE_SPECIMEN)
    fun release_specimen() = setState(State.RELEASE_SPECIMEN)
    fun spec_preload() = setState(State.SPEC_PRELOAD)
    fun score_front() = setState(State.SCORE_FRONT)
    fun sweep() = setState(State.SWEEP)

    fun write() {
        extensionOne.write()
        extensionTwo.write()
        roll.write()
        pitch.write()
        subPitch.write()
        claw.write()
    }
}