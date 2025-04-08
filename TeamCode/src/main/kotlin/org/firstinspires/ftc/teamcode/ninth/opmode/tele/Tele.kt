package org.firstinspires.ftc.teamcode.ninth.opmode.tele

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.util.hardware.SMGamepad
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.State.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Roll.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Color.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift.Preset.*
import kotlin.math.abs

@TeleOp(name="ALL Colors Tele")
open class Tele : LinearOpMode() {
    open val goodColors = setOf(YELLOW, RED, BLUE)
    open val badColors = setOf(NONE)

    override fun runOpMode() {
        val driver = SMGamepad(gamepad1)
        val operator = SMGamepad(gamepad2)

        val drivetrain = Drivetrain(hardwareMap)
        // TODO: readd tele
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage).coerceAtLeast(1.0))
        val sampler = Sampler(hardwareMap)

        val COLLISION_WAIT = 0.85
        val PRIME_WAIT = 0.08
        val GRAB_WAIT = 0.08
        val SCORE_WAIT = 0.2

        var manual = false
        // TODO: SET TO LOW FOR POST-AUTO
        lift.setPreset(BOTTOM)
        var ITSCLIMBINTIME = false

        var speedDecrease = 0.0
        var turnDecrease = 0.0

        val samplerFSM = StateMachineBuilder()
            .state(STOW)
            .onEnter {
                speedDecrease = 0.0
                turnDecrease = 0.0
            }
            .transition({ driver.lt.rising && lift.getPreset() == BOTTOM }, EXTING_SAMP, { sampler.setRoll(R0) })
            .transition({ driver.rt.rising && lift.getPreset() == BOTTOM }, EXTING_SAMP, { sampler.setRoll(R90) })
            .transition({ driver.lb.rising && lift.getPreset() == BOTTOM }, EXTING_SAMP, { sampler.setRoll(R45) })
            .transition({ driver.rb.rising && lift.getPreset() == BOTTOM }, EXTING_SAMP, { sampler.setRoll(RCW45) })
            .transition({ driver.a.rising  && lift.getPreset() == BOTTOM }, EXTING_SPEC)

            .state(EXTING_SAMP)
            .minimumTransitionTimed(0.6)
            .transitionTimed(COLLISION_WAIT)
            .state(EXT_SAMP)
            .onEnter { speedDecrease = 2.5; turnDecrease = 2.5 }
            .loop {
                when {
                    driver.rt.rising -> sampler.setRoll(if (sampler.getRoll() == R90.pos) R0 else R90)
                    driver.lb.rising -> { sampler.setRoll(R45) }
                    driver.rb.rising -> { sampler.setRoll(RCW45) }
                }
            }
            .transition({ driver.lt.rising }, PRIME_SAMP)
            .transition({ driver.x.rising }, STOW)
            .transition({ operator.x.rising }, STOW)

            // speed_decrease = 0.75
            // turn_decrease = 1.5
            .state(PRIME_SAMP)
            .transitionTimed(PRIME_WAIT)
            .state(GRAB_SAMP)
            .transitionTimed(GRAB_WAIT, PICKED)

            .state(PICKED)
            .onEnter{ speedDecrease = 0.0; turnDecrease = 0.0 }
            // TODO: add override in case of sensor failure
            .transition({ sampler.color in goodColors }, HOLD_SAMP)
            .transition({ sampler.color in badColors }, EXT_SAMP)
            .transition({ driver.lt.rising }, HOLD_SAMP)
            .transition({ driver.rt.rising }, EXT_SAMP)
            .transition({ operator.x.rising }, STOW)

            .state(HOLD_SAMP)
            .transition({ driver.lt.rising }, MOVE_SCORE_SAMP)
            .transition(
                {
                    (lift.getPreset() == Lift.Preset.SAMP_HIGH ||
                            lift.getPreset() == Lift.Preset.SAMP_LOW) &&
                        abs(lift.getHeight() - lift.getPreset().height) < 3.0
                },
                MOVE_SCORE_SAMP,
            )
            .transition({ driver.rt.rising }, OBS_EXT_SAMP)
            .transition({ operator.x.rising }, STOW)

            .state(MOVE_SCORE_SAMP)
            .transitionTimed(0.4)
            .state(PREP_SCORE_SAMP)
            .transition({ driver.lt.rising }, SCORE_SAMP)
            .transition({ operator.x.rising }, STOW)
            .state(SCORE_SAMP)
            .onEnter { speedDecrease = 0.0; turnDecrease = 0.0 }
            .transitionTimed(SCORE_WAIT, STOW)
            .transition({ operator.x.rising }, STOW)

            .state(OBS_EXT_SAMP)
            .transition({ driver.lt.rising }, OBS_DROP_SAMP)
            .transition({ operator.x.rising }, STOW)
            .state(OBS_DROP_SAMP)
            .transitionTimed(0.2, STOW)



            .state(EXTING_SPEC)
            .transitionTimed(COLLISION_WAIT)
            .state(PRIME_SPEC)
            .onEnter { speedDecrease = 0.0; turnDecrease = 1.5 }
            .transition({ driver.lt.rising }, GRAB_SPEC)
            .transition({ operator.x.rising}, STOW)
            .state(GRAB_SPEC)
            .transitionTimed(GRAB_WAIT)
            .state(RAM_SPEC)
            .onEnter { lift.setPreset(SPEC_HIGH) }
            .transition({ driver.lt.rising }, RELEASE_SPEC)
            .transition({ operator.x.rising}, STOW)
            .state(RELEASE_SPEC)
            .transition({ driver.lt.rising }, STOW)
            .transition({ operator.a.rising}, STOW)

            .build()

        var lastState = STOW

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        samplerFSM.start()
        while (opModeIsActive()) {
            driver.update()
            operator.update()

            // drive
            // TODO: disable during PTO
            drivetrain.setEffort(
                -driver.lsy.sq / (1 + speedDecrease),
                -driver.lsx.sq / (1 + speedDecrease),
                -driver.rsx.sq / (1.5 + turnDecrease),
            )

            // lift
            lift.read()
            if (operator.a.rising && lift.getPreset() != BOTTOM) {
                lift.setPreset(BOTTOM)
                speedDecrease = 0.0
            }
            if (samplerFSM.state == HOLD_SAMP || samplerFSM.state == PREP_SCORE_SAMP) {
                if (operator.b.rising) { lift.setPreset(SAMP_LOW) }
                if (operator.y.rising) { lift.setPreset(SAMP_HIGH) }
                if (lift.getPreset() != BOTTOM) {
                    speedDecrease = 2.0
                }
            }

            if (operator.lb.pressed && operator.rb.pressed && operator.up.rising) {
                sampler.incrementPitch()
            }
            if (operator.lb.pressed && operator.rb.pressed && operator.down.rising) {
                sampler.decrementPitch()
            }
            if (operator.guide.rising) { manual = !manual }
            // TODO: readd this stuff
            // how tf did this work before???
            if (!manual) {
                if (!ITSCLIMBINTIME) {
                    lift.updateProfiled(lift.getHeight(), debug = telemetry)
                } else {
                    lift.pto2CLIMB(lift.getHeight())
                }
            } else {
                lift.setEffort(-operator.lsy.pos + 0.2)
                if (gamepad2.dpad_down && -operator.rsy.pos < -0.8) {
                    lift.resetEncoder()
                }
            }

            // TODO: utilize a rising edge here?
            lastState = samplerFSM.state as Sampler.State
            samplerFSM.update()
            if (samplerFSM.state != lastState) {
                sampler.state = samplerFSM.state as Sampler.State
            }
            sampler.updateProfiled()

            drivetrain.write()
            lift.write()
            sampler.write()

            telemetry.addLine("lift ctrls, samp & spec - g2 a, b, y")
            telemetry.addLine("reset to stow - g2 x")
            telemetry.addLine("manual lift - g2 middle guide button")
            telemetry.addLine("lift reset (in manual) - g2 dpad up + left stick up")
            telemetry.addLine("pitch adjust - g2 both bumpers + dpad up/down")
            telemetry.addLine(" ")
            telemetry.addData("height", lift.getHeight())
            telemetry.addData("state", samplerFSM.state)
            telemetry.update()
        }
    }
}
