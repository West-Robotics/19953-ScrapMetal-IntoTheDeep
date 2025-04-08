package org.firstinspires.ftc.teamcode.ninth.robot.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.MPConstraints
import com.scrapmetal.util.control.motionProfile
import com.scrapmetal.util.hardware.SMServo
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Claw.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Roll.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Color.*

// TODO: add different scoring orientations
class Sampler(hardwareMap: HardwareMap) {
    init { rollAng = R0.pos }
    // TODO: reverse servos to their appropriate directions
    // TODO: should we reconsider this servo naming?
    private val extensionOne = SMServo(hardwareMap, "frontExt", State.STOW.linkage.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val extensionTwo = SMServo(hardwareMap, "backExt", State.STOW.linkage.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    // TODO: Make hardware private again
    private val proximal = SMServo(hardwareMap, "proximal", State.STOW.proximal.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val distal = SMServo(hardwareMap, "distal", State.STOW.distal.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val roll = SMServo(hardwareMap, "roll", State.STOW.roll.invoke(), Servo.Direction.FORWARD, SMServo.ModelPWM.AXON)
    private val claw = SMServo(hardwareMap, "claw", State.STOW.claw.pos, Servo.Direction.REVERSE, SMServo.ModelPWM.AXON)
    private val color0 = hardwareMap.digitalChannel.get("color0")
    private val color1 = hardwareMap.digitalChannel.get("color1")

    private var mpStart = State.EXT_SAMP.proximal.pos
    private var mpEnd = State.EXT_SAMP.proximal.pos
    private val mpTimer = ElapsedTime()
    private var pitchOffset = 0.0

    var state: State = State.STOW
        set(value) {
            field = value
            claw.position = value.claw.pos
            roll.position = value.roll.invoke()
            distal.position = value.distal.pos
            mpStart = proximal.position
            mpEnd = value.proximal.pos
            mpTimer.reset()
            extensionOne.position = value.linkage.pos
            extensionTwo.position = value.linkage.pos
        }

    enum class Color {
        YELLOW,
        RED,
        BLUE,
        NONE,
    }

    val color
        get() = when (Pair(color0.state, color1.state)) {
            Pair(true,  true)  -> YELLOW
            Pair(false, true)  -> RED
            Pair(true,  false) -> BLUE
            else               -> NONE
        }

    // TODO: make roll a lambda to deal with different roll?
    enum class State(
        val proximal: Prox,
        val distal: Dist,
        val roll: () -> Double,
        val claw: Claw,
        val linkage: Lkg,
    ) {
        STOW            (Prox.STOW, Dist.RET,  { R0.pos },    OPEN,  Lkg.RET),
        EXTING_SAMP     (Prox.RET,  Dist.EXT,  { R0.pos },    OPEN,  Lkg.EXT),
        EXT_SAMP        (Prox.STOW, Dist.EXT,  { rollAng },   OPEN,  Lkg.EXT),
        PRIME_SAMP      (Prox.EXT,  Dist.EXT,  { rollAng },   OPEN,  Lkg.EXT),
        GRAB_SAMP       (Prox.EXT,  Dist.EXT,  { rollAng },   CLOSE, Lkg.EXT),
        PICKED          (Prox.RET,  Dist.EXT,  { R0.pos },    CLOSE, Lkg.EXT),
        HOLD_SAMP       (Prox.STOW, Dist.RET,  { R90.pos },   CLOSE, Lkg.RET),
        MOVE_SCORE_SAMP (Prox.OUT,  Dist.RET,  { R90.pos },   CLOSE, Lkg.RET),
        PREP_SCORE_SAMP (Prox.OUT,  Dist.RET,  { R0.pos },    CLOSE, Lkg.RET),
        SCORE_SAMP      (Prox.OUT,  Dist.RET,  { R0.pos },    OPEN,  Lkg.RET),
        OBS_EXT_SAMP    (Prox.STOW, Dist.RET,  { R90.pos },   CLOSE, Lkg.EXT),
        OBS_DROP_SAMP   (Prox.STOW, Dist.EXT,  { R0.pos },    OPEN,  Lkg.EXT),

        EXTING_SPEC     (Prox.RET,  Dist.EXT,  { R90.pos },   OPEN,  Lkg.EXT),
        PRIME_SPEC      (Prox.FLAT, Dist.SPEC, { R90.pos },   OPEN,  Lkg.EXT),
        GRAB_SPEC       (Prox.FLAT, Dist.SPEC, { R90.pos },   CLOSE, Lkg.EXT),
        RAM_SPEC        (Prox.RAM,  Dist.RAM,  { RCW90.pos }, CLOSE, Lkg.RET),
        RELEASE_SPEC    (Prox.RAM,  Dist.RAM,  { RCW90.pos }, OPEN,  Lkg.RET),

        DEBUG           (Prox.VERT, Dist.FLAT, { R0.pos }, OPEN, Lkg.RET),
    }

    enum class Prox(val pos: Double) {
        EXT  (0.46),
        FLAT (0.47),
        STOW (0.50),
        RET  (0.56),
        OUT  (0.72),
        RAM  (0.92),

        VERT (0.67), // debug
    }

    enum class Dist(val pos: Double) {
        EXT  (0.64),
        SPEC (0.70),
        RAM  (0.57),
        RET  (0.98),

        FLAT (0.86),
    }

    // Names reference the angle of the sample so we don't have to rotate 90 degrees in our head every time
    enum class Roll(val pos: Double) {
        R0    (0.36),
        R45   (0.23),
        RCW45 (0.00),
        R90   (0.10),
        RCW90 (0.62), // for specs
    }

    enum class Claw(val pos: Double) {
        OPEN  (0.94),
        CLOSE (0.74),
        RAM   (0.72),
    }

    enum class Lkg(val pos: Double) {
        RET (0.03),
        EXT (0.60),
    }

    // TODO: remove retracting
    fun updateProfiled() {
        proximal.position = motionProfile(
            MPConstraints(
                start = mpStart,
                end = mpEnd,
                accel = 82.0,
                decel = if (mpEnd == Prox.EXT.pos) 64.0 else 6.0,
                vLimit = 36.0,
            ),
            mpTimer.seconds(),
        ).s + pitchOffset
    }

    companion object {
        var rollAng = R0.pos
    }

    fun setRoll(state: Roll) {
        rollAng = state.pos
        roll.position = rollAng
    }

    fun setRoll(ang: Double) {
        rollAng = R0.pos - ang.coerceIn(-90.0, 90.0) / 355.0
        roll.position = rollAng
    }

    fun getRoll() = rollAng

    // TODO: maybe increase servo caching precision
    fun incrementPitch() { pitchOffset += 0.01 }

    fun decrementPitch() { pitchOffset -= 0.01 }

    fun write() {
        extensionOne.write()
        extensionTwo.write()
        proximal.write()
        distal.write()
        roll.write()
        claw.write()
    }
}