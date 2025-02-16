package org.firstinspires.ftc.teamcode.ninth.opmode.tele

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.scrapmetal.util.hardware.SMAnalog
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "SpecTele")

class SpecTele: LinearOpMode() {
    // TODO: test reset, manual, charge bot, test positions of lift and intake, pow turn
    enum class SamplerState {
        STOW,
        EXTEND_SAMPLE,
        GRAB_SAMPLE,
        GRAB_SAMPLE_LEFT_SIDE,
        GRAB_SAMPLE_RIGHT_SIDE,
        SPIT_SAMPLE,
        HOLD_SAMPLE,
        PREPARE_TO_SCORE_SAMPLE,
        SCORE_SAMPLE,
        EXTEND_SPECIMEN,
        GRAB_SPECIMEN,
        LIFT_SPECIMEN,
        HOLD_SPECIMEN,
//        PREPARE_TO_SCORE_SPECIMEN,
        SCORE_SPECIMEN,
//        RELEASE_SPECIMEN,
        RAISE_CLIMB,
        PULL_CLIMB,
    }
    val sampleWait = 0.4

    var specHeights = false
    var sampHeights = false

    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage).coerceAtLeast(1.0), drivetrain)
        val sampler = Sampler(hardwareMap)
        val intakeSpeed = SMAnalog(hardwareMap, "intakeEnc")

        val retract_wait = 2.0

        var manual = false
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        // TODO: SET TO LOW FOR POST-AUTO
        lift.setPreset(Lift.Preset.INIT_POST_AUTO)
        var ITSCLIMBINTIME = false

        var speed_decrease = 0.0
        var turn_decrease = 0.0

        val fsm = StateMachineBuilder()
            .state(SamplerState.STOW)
            .onEnter {
                sampler.stow()
                speed_decrease = 0.0
                turn_decrease = 0.0
                sampHeights = false
                specHeights = false
            }
            .loop { sampler.stow() }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8
                        && lift.getPreset() == Lift.Preset.BOTTOM },
                SamplerState.EXTEND_SAMPLE,
            )
            .transition(
                { currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8
                        && (lift.getPreset() == Lift.Preset.BOTTOM || lift.getPreset() == Lift.Preset.SPEC_INTAKE) },
                SamplerState.EXTEND_SPECIMEN,
            )
            .transition(
                { currentGamepad2.left_trigger > 0.8 && currentGamepad2.right_trigger > 0.8 && currentGamepad2.dpad_up },
                SamplerState.RAISE_CLIMB,
            )

            .state(SamplerState.RAISE_CLIMB)
            .onEnter { lift.setPreset(Lift.Preset.RAISE_CLIMB) }
            .transition(
                { currentGamepad2.left_trigger > 0.8 && currentGamepad2.right_trigger > 0.8 && currentGamepad2.dpad_down },
                SamplerState.PULL_CLIMB,
            )

            .state(SamplerState.PULL_CLIMB)
            .onEnter { lift.pto1FREEZE(); lift.pto1ENGAGE() }
            .afterTime(0.30) { ITSCLIMBINTIME = true }

//          ░██████╗░█████╗░███╗░░░███╗██████╗░██╗░░░░░███████╗
//          ██╔════╝██╔══██╗████╗░████║██╔══██╗██║░░░░░██╔════╝
//          ╚█████╗░███████║██╔████╔██║██████╔╝██║░░░░░█████╗░░
//          ░╚═══██╗██╔══██║██║╚██╔╝██║██╔═══╝░██║░░░░░██╔══╝░░
//          ██████╔╝██║░░██║██║░╚═╝░██║██║░░░░░███████╗███████╗
//          ╚═════╝░╚═╝░░╚═╝╚═╝░░░░░╚═╝╚═╝░░░░░╚══════╝╚══════╝

            .state(SamplerState.EXTEND_SAMPLE)
            .onEnter {
                sampler.extend()
                speed_decrease = 0.0
                turn_decrease = 1.5
            }
            .loop { sampler.extend() }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.GRAB_SAMPLE,
            )
            .transition(
                { currentGamepad1.left_bumper && !previousGamepad1.left_bumper },
                SamplerState.GRAB_SAMPLE_LEFT_SIDE,
            )
            .transition(
                { currentGamepad1.right_bumper && !previousGamepad1.right_bumper },
                SamplerState.GRAB_SAMPLE_RIGHT_SIDE,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.GRAB_SAMPLE)
            .onEnter {
                sampler.grab_sample()
                speed_decrease = 0.75
                turn_decrease = 1.5
            }
            .loop { sampler.grab_sample() }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.HOLD_SAMPLE,
                { speed_decrease = 0.0; turn_decrease = 0.0 },
            )
            .minimumTransitionTimed(0.25, 1)
            .transition(
                { currentGamepad1.left_bumper && !previousGamepad1.left_bumper },
                SamplerState.GRAB_SAMPLE_LEFT_SIDE,
            )
            .transition(
                { currentGamepad1.right_bumper && !previousGamepad1.right_bumper },
                SamplerState.GRAB_SAMPLE_RIGHT_SIDE,
            )
            .transition(
                { currentGamepad1.x && !previousGamepad1.x },
                SamplerState.SPIT_SAMPLE,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.GRAB_SAMPLE_LEFT_SIDE)
            .onEnter { sampler.grab_sample_left_side() }
            .loop { sampler.grab_sample_left_side() }
            .transition(
                { currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8 },
                SamplerState.GRAB_SAMPLE,
            )
            .transition(
                { currentGamepad1.right_bumper && !previousGamepad1.right_bumper },
                SamplerState.GRAB_SAMPLE_RIGHT_SIDE,
            )
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.HOLD_SAMPLE,
                { speed_decrease = 0.0; turn_decrease = 0.0 },
            )
            .minimumTransitionTimed(0.25, 3)
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.GRAB_SAMPLE_RIGHT_SIDE)
            .onEnter { sampler.grab_sample_right_side() }
            .loop { sampler.grab_sample_right_side() }
            .transition(
                { currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8 },
                SamplerState.GRAB_SAMPLE,
            )
            .transition(
                { currentGamepad1.left_bumper && !previousGamepad1.left_bumper },
                SamplerState.GRAB_SAMPLE_LEFT_SIDE,
            )
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.HOLD_SAMPLE,
                { speed_decrease = 0.0; turn_decrease = 0.0 },
            )
            .minimumTransitionTimed(0.25, 3)
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.SPIT_SAMPLE)
            // NOTE: maybe we should make the various preset speed decreases variables
            .onEnter { sampler.spit(); speed_decrease = 0.75; turn_decrease = 1.5 }
            .transitionTimed(sampleWait, SamplerState.GRAB_SAMPLE)
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW
            )

            .state(SamplerState.HOLD_SAMPLE)
            .onEnter { sampler.hold(); sampHeights = true; turn_decrease = 0.0 }
            .loop { sampler.hold() }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.PREPARE_TO_SCORE_SAMPLE,
            )
            .transition(
                {
                    (lift.getPreset() == Lift.Preset.SAMP_HIGH ||
                            lift.getPreset() == Lift.Preset.SAMP_LOW) &&
                            abs(lift.getHeight() - lift.getPreset().height) < 0.75
                },
                SamplerState.PREPARE_TO_SCORE_SAMPLE,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.PREPARE_TO_SCORE_SAMPLE)
            .onEnter { sampler.prepare_to_score_sample() }
            .loop { sampler.prepare_to_score_sample() }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.SCORE_SAMPLE,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.SCORE_SAMPLE)
            .onEnter { sampler.score_sample(); sampHeights = true
                // WARNING: this should probably be changed so that speed decrease only resets once lift lowers
                speed_decrease = 0.0
                turn_decrease = 0.0
            }
            .transitionTimed(sampleWait, SamplerState.STOW)
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

//          ░██████╗██████╗░███████╗░█████╗░██╗███╗░░░███╗███████╗███╗░░██╗
//          ██╔════╝██╔══██╗██╔════╝██╔══██╗██║████╗░████║██╔════╝████╗░██║
//          ╚█████╗░██████╔╝█████╗░░██║░░╚═╝██║██╔████╔██║█████╗░░██╔██╗██║
//          ░╚═══██╗██╔═══╝░██╔══╝░░██║░░██╗██║██║╚██╔╝██║██╔══╝░░██║╚████║
//          ██████╔╝██║░░░░░███████╗╚█████╔╝██║██║░╚═╝░██║███████╗██║░╚███║
//          ╚═════╝░╚═╝░░░░░╚══════╝░╚════╝░╚═╝╚═╝░░░░░╚═╝╚══════╝╚═╝░░╚══╝

            // TODO: speed decrease? auto positioning?
            .state(SamplerState.EXTEND_SPECIMEN)
            .onEnter { sampler.extend(); lift.setPreset(Lift.Preset.BOTTOM) }
            .transitionTimed(0.25, SamplerState.GRAB_SPECIMEN)
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.GRAB_SPECIMEN)
            .onEnter { sampler.grab_specimen() }
            .loop { sampler.grab_specimen() }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.LIFT_SPECIMEN
            )

            .state(SamplerState.LIFT_SPECIMEN)
            .onEnter { sampler.lift_specimen() }
            .loop { sampler.updateProfiled() }
            .transitionTimed(0.4, SamplerState.HOLD_SPECIMEN)

            .state(SamplerState.HOLD_SPECIMEN)
            .onEnter { sampler.hold_specimen(); specHeights = true }
            .loop { sampler.updateProfiled() }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8
                        && (lift.getPreset() == Lift.Preset.SPEC_HIGH || lift.getPreset() == Lift.Preset.SPEC_LOW) },
                SamplerState.SCORE_SPECIMEN,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

//            .state(SamplerState.PREPARE_TO_SCORE_SPECIMEN)
//            .onEnter { sampler.prepare_to_score_specimen() }
//            .loop { sampler.updateProfiled() }
//            .transition(
//                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8
//                        && lift.getPreset() == Lift.Preset.SPEC_LOW },
//                SamplerState.SCORE_SPECIMEN_LOW,
//            )
//            .transition(
//                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8
//                        && lift.getPreset() == Lift.Preset.SPEC_HIGH },
//                SamplerState.SCORE_SPECIMEN_HIGH,
//            )
//            .transition(
//                { currentGamepad2.x && !previousGamepad2.x },
//                 how would we eject an unwanted spec?
//                SamplerState.STOW,
//            )

            .state(SamplerState.SCORE_SPECIMEN)
            .onEnter { sampler.dip_specimen() }
            .loop { sampler.updateProfiled() }
            .transitionTimed(0.8)
            .waitState(0.8)
            .onEnter { sampler.retract_specimen() }
            .loop { sampler.updateProfiled() }
            .waitState(0.8)
            .onEnter { sampler.score_specimen(); lift.setPreset(Lift.Preset.SPEC_HIGH_SCORE) }
            .waitState(0.8)
            .onEnter { sampler.release_specimen() }
            .waitState(0.8, SamplerState.STOW)
            .onEnter { sampler.stow() }
            .onExit { lift.setPreset(Lift.Preset.SPEC_MID) }

//            .state(SamplerState.RELEASE_SPECIMEN)
//            .onEnter { sampler.prepare_to_score_specimen() }
//            .transitionTimed(1.0)
//            .waitState(1.0, SamplerState.STOW)
//            .onEnter { sampler.score_specimen() }
//            .onExit { lift.setPreset(Lift.Preset.SPEC_MID) }
            .build()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        fsm.start()
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            // drive
            // TODO: disable during PTO
            drivetrain.setEffort(
                -sign(gamepad1.left_stick_y.toDouble()) * gamepad1.left_stick_y.toDouble().pow(2) / (1 + speed_decrease),
                -sign(gamepad1.left_stick_x.toDouble()) * gamepad1.left_stick_x.toDouble().pow(2) / (1 + speed_decrease),
                -sign(gamepad1.right_stick_x.toDouble()) * gamepad1.right_stick_x.toDouble().pow(2) / (1.5 + turn_decrease),
            )

            // lift
            lift.read()
            if (currentGamepad2.a && !previousGamepad2.a) { lift.setPreset(Lift.Preset.BOTTOM) ; speed_decrease = 0.0 }
            if (currentGamepad2.b && !previousGamepad2.b && sampHeights) { lift.setPreset(Lift.Preset.SAMP_LOW) ; speed_decrease = 2.0 }
            if (currentGamepad2.y && !previousGamepad2.y && sampHeights) { lift.setPreset(Lift.Preset.SAMP_HIGH) ; speed_decrease = 2.0 }
            if (currentGamepad2.b && !previousGamepad2.b && specHeights) { lift.setPreset(Lift.Preset.SPEC_LOW) ; speed_decrease = 2.0 }
            if (currentGamepad2.y && !previousGamepad2.y && specHeights) { lift.setPreset(Lift.Preset.SPEC_HIGH) ; speed_decrease = 2.0 }

            if (
                currentGamepad2.left_bumper && currentGamepad2.right_bumper &&
                currentGamepad2.dpad_up && !previousGamepad2.dpad_up
            ) {
                sampler.incrementPitch()
            }
            if (
                currentGamepad2.left_bumper && currentGamepad2.right_bumper &&
                currentGamepad2.dpad_down && !previousGamepad2.dpad_down
            ) {
                sampler.decrementPitch()
            }
            if (currentGamepad2.start && !previousGamepad2.start) { manual = !manual }
            if (!manual) {
                if (!ITSCLIMBINTIME) {
                    when (lift.getPreset()) {
                        Lift.Preset.SPEC_HIGH_SCORE, Lift.Preset.SPEC_LOW_SCORE ->
                            lift.updatePid(lift.getHeight())
                        else -> lift.updateProfiled(lift.getHeight(), debug = telemetry)
                    }
                } else {
                    lift.pto2CLIMB(lift.getHeight())
                }
            } else {
                lift.setEffort(-gamepad2.left_stick_y + 0.2)
                if (gamepad2.dpad_down && -gamepad2.right_stick_y < -0.8) {
                    lift.resetEncoder()
                }
            }

            fsm.update()

            drivetrain.write()
            lift.write()
            sampler.write()

            telemetry.addLine("regular pathway - g1 left trigger")
            telemetry.addLine("alt pathway - g1 right trigger")
            telemetry.addLine("lift ctrls, samp & spec - g2 a, b, y")
            telemetry.addLine("reset to stow - g2 x")
            telemetry.addLine("manual lift - g2 start")
            telemetry.addLine("lift reset (in manual) - g2 dpad up + left stick up")
            telemetry.addLine("               ")
            telemetry.addData("height", lift.getHeight())
            telemetry.addData("state", fsm.state)
            telemetry.update()
        }
    }
}