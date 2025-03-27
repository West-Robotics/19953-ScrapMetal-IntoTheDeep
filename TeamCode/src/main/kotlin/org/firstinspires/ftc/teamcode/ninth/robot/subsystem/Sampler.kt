package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.MPConstraints
import com.scrapmetal.util.control.motionProfile
import com.scrapmetal.util.hardware.SMServo
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Claw.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Roll.*

class Sampler(hardwareMap: HardwareMap) {
    init { rollState = R0 }
    // TODO: reverse servos to their appropriate directions
    // TODO: should we reconsider this servo naming?
    private val extensionOne = SMServo(hardwareMap, "frontExt", State.STOW.linkage.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val extensionTwo = SMServo(hardwareMap, "backExt", State.STOW.linkage.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    // TODO: Make hardware private again
    private val proximal = SMServo(hardwareMap, "proximal", State.STOW.proximal.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val distal = SMServo(hardwareMap, "distal", State.STOW.distal.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val roll = SMServo(hardwareMap, "roll", State.STOW.roll.invoke().pos, Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val claw = SMServo(hardwareMap, "claw", State.STOW.claw.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)

    companion object {
        private var rollState = R0
    }
    private var mpStart = State.EXTEND.proximal.pos
    private var mpEnd = State.EXTEND.proximal.pos
    private val mpTimer = ElapsedTime()
    private var pitchOffset = 0.0

    // TODO: make roll a lambda to deal with different roll?
    enum class State(
        val proximal: Prox,
        val distal: Dist,
        val roll: () -> Roll,
        val claw: Claw,
        val linkage: Lkg,
    ) {
        STOW            (Prox.RET, Dist.RET,  { R0 },        OPEN,  Lkg.RET),
        EXTENDING       (Prox.RET, Dist.EXT,  { R0 },        OPEN,  Lkg.EXT),
        EXTEND          (Prox.FLAT, Dist.EXT, { rollState }, OPEN,  Lkg.EXT),
        PRIME_SAMP      (Prox.EXT,  Dist.EXT, { rollState }, OPEN,  Lkg.EXT),
        GRAB_SAMP       (Prox.EXT,  Dist.EXT, { rollState }, CLOSE, Lkg.EXT),
        PICKED          (Prox.RET, Dist.EXT,  { R0 },        CLOSE, Lkg.EXT),
        HOLD_SAMP       (Prox.RET, Dist.RET,  { R0 },        CLOSE, Lkg.RET),
        // TODO: add different scoring orientations
        PREP_SCORE_SAMP (Prox.OUT,  Dist.RET, { R0 },        CLOSE, Lkg.RET),
        SCORE_SAMP      (Prox.OUT,  Dist.RET, { R0 },        OPEN,  Lkg.RET),

        SPIT            (Prox.EXT,  Dist.EXT,  { R0 }, OPEN,  Lkg.EXT),
        HOLD            (Prox.FLAT, Dist.RET,  { R0 }, CLOSE, Lkg.RET),
        GRAB_SPEC       (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        LIFT_SPEC       (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        HOLD_SPEC       (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        DIP_SPEC        (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        RETRACT_SPEC    (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        SCORE_SPEC      (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        RELEASE_SPEC    (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        SCORE_FRONT     (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        SWEEP           (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        PREP_SCORE_SPEC (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
        SPEC_PRELOAD    (Prox.FLAT, Dist.FLAT, { R0 }, OPEN, Lkg.EXT),
    }

    enum class Prox(val pos: Double) {
        EXT  (0.45),
        FLAT (0.48),
        RET  (0.53),
        OUT  (0.68),
    }

    enum class Dist(val pos: Double) {
        EXT  (0.63),
        FLAT (0.89),
        RET  (1.00),
    }

    // Names reference the angle of the sample so we don't have to rotate 90 degrees in our head every time
    enum class Roll(val pos: Double) {
        R0    (0.36), // -> R90
        R45   (0.23),
        RCW45 (0.00),
        R90   (0.10), // -> R0
    }

    enum class Claw(val pos: Double) {
        OPEN  (0.96),
        CLOSE (0.74),
    }

    enum class Lkg(val pos: Double) {
        RET (0.03),
        EXT (0.60),
    }

    fun setState(state: State) {
        claw.position = state.claw.pos
        roll.position = state.roll.invoke().pos
        distal.position = state.distal.pos
        mpStart = proximal.position
        mpEnd = state.proximal.pos
        mpTimer.reset()
        extensionOne.position = state.linkage.pos
        extensionTwo.position = state.linkage.pos
    }

    fun setRoll(state: Roll) { rollState = state }

    // TODO: remove retracting
    fun updateProfiled(retracting: Boolean = false) {
        proximal.position = motionProfile(
            MPConstraints(
                start = mpStart,
                end = mpEnd,
                accel = 36.0,
                decel = 8.0,
                vLimit = 36.0,
            ),
            mpTimer.seconds(),
        ).s + pitchOffset
    }

    // TODO: maybe increase servo caching precision
    fun incrementPitch() { pitchOffset += 0.01 }

    fun decrementPitch() { pitchOffset -= 0.01 }

    fun extend() = setState(State.EXTEND)
    fun grab_sample() = setState(State.PRIME_SAMP)
    fun grab_sample_left_side() = setState(State.PRIME_SAMP)
    fun grab_sample_right_side() = setState(State.PRIME_SAMP)
    fun grab_sample_side() = setState(State.PRIME_SAMP)
    fun spit() = setState(State.SPIT)
    fun stow() = setState(State.STOW)
    fun hold() = setState(State.HOLD)
    fun hold_sampele() = setState(State.HOLD_SAMP)
    fun grab_specimen() = setState(State.GRAB_SPEC)
    fun lift_specimen() = setState(State.LIFT_SPEC)
    fun hold_specimen() = setState(State.HOLD_SPEC)
    fun score_sample() = setState(State.SCORE_SAMP)
    fun prepare_to_score_sample() = setState(State.PREP_SCORE_SAMP)
    fun prepare_to_score_specimen() = setState(State.PREP_SCORE_SPEC)
    fun dip_specimen() = setState(State.DIP_SPEC)
    fun dip_specimen_fast() = setState(State.DIP_SPEC)
    fun retract_specimen() = setState(State.RETRACT_SPEC)
    fun score_specimen() = setState(State.SCORE_SPEC)
    fun release_specimen() = setState(State.RELEASE_SPEC)
    fun spec_preload() = setState(State.SPEC_PRELOAD)
    fun score_front() = setState(State.SCORE_FRONT)
    fun sweep() = setState(State.SWEEP)

    fun write() {
        extensionOne.write()
        extensionTwo.write()
        proximal.write()
        distal.write()
        roll.write()
        claw.write()
    }
}