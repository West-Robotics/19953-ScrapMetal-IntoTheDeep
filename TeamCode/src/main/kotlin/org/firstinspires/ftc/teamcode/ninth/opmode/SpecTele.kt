package org.firstinspires.ftc.teamcode.ninth.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.scrapmetal.util.hardware.SMAnalog
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "SpecTele")

class SpecTele: LinearOpMode() {
    // TODO: test reset, manual, charge bot, test positions of lift and intake, pow turn
    enum class SamplerState {
        STOW,
        EXTEND,
        GRAB_SAMPLE,
        GRAB_SAMPLE_SIDE,
        GRAB_SPECIMEN,
        SPIT,
        HOLD_SAMPLE,
        HOLD_SPECIMEN,
        SCORE_SAMPLE,
        PREPARE_TO_SCORE_SPECIMEN,
        SCORE_SPECIMEN_LOW,
        SCORE_SPECIMEN_HIGH,
    }
    val sampleWait = 0.5

    var specHeights = false
    var sampHeights = false

    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)
        val intakeSpeed = SMAnalog(hardwareMap, "intakeEnc")

        val intake_stalled = 2.0
        val retract_wait = 2.0

        var manual = false
//        lift.setPreset(Lift.Preset.LOW)
        lift.setPreset(Lift.Preset.BOTTOM)

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
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8
                        && lift.getPreset() == Lift.Preset.BOTTOM },
                SamplerState.EXTEND,
            )
            .transition(
                { currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8
                        && lift.getPreset() == Lift.Preset.BOTTOM },
                SamplerState.GRAB_SPECIMEN,
            )

            .state(SamplerState.EXTEND)
            .onEnter {
                sampler.extend()
                speed_decrease = 0.0
                turn_decrease = 1.5
            }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.GRAB_SAMPLE,
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
            .transition(
                { intakeSpeed.speed < intake_stalled },
                SamplerState.HOLD_SAMPLE,
                { speed_decrease = 0.0 },
            )
            .minimumTransitionTimed(retract_wait, 1)
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.HOLD_SAMPLE,
                { speed_decrease = 0.0 },
            )
            .minimumTransitionTimed(0.25, 2)
            .transition(
                { currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8 },
                SamplerState.GRAB_SAMPLE_SIDE,
            )
            .transition(
                { currentGamepad1.right_bumper && !previousGamepad1.right_bumper },
                SamplerState.SPIT,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.GRAB_SAMPLE_SIDE)
            .onEnter { sampler.grab_sample_side() }
            .transition(
                { currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8 },
                SamplerState.GRAB_SAMPLE,
            )
            .transition(
                { intakeSpeed.speed < intake_stalled },
                SamplerState.HOLD_SAMPLE,
            )
            .minimumTransitionTimed(retract_wait, 2)
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.HOLD_SAMPLE,
                { speed_decrease = 0.0 },
            )
            .minimumTransitionTimed(0.25, 3)
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.GRAB_SPECIMEN)
            // WARNING: .setPreset() can't be used in a loop
            .onEnter { sampler.extend(); lift.setPreset(Lift.Preset.SPEC_INTAKE) }
            .afterTime(1.0) { sampler.grab_specimen() }
            .transition(
                { intakeSpeed.speed < intake_stalled },
                SamplerState.HOLD_SPECIMEN,
            )
            .minimumTransitionTimed(retract_wait, 1)
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.HOLD_SPECIMEN,
            )
            .minimumTransitionTimed(0.25, 2)
            // QUESTION: should this transition also include lowering the lift?
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.HOLD_SPECIMEN)
            // QUESTION: i don't think we need the .setPreset() here since we're already at the position
            // (and it's not the looping function which is .updateProfiled())
            // perhaps this state should first raise the lift, then after some time (using .afterTime)
            // retract to the hold position? also, does the specimen fit through the robot sideways?
            // if not, we might need a special spec hold position with roll at 90 deg
            .onEnter { sampler.hold_specimen(); specHeights = true }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.PREPARE_TO_SCORE_SPECIMEN,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.SPIT)
            // NOTE: maybe we should make the various preset speed decreases variables
            .onEnter { sampler.spit(); speed_decrease = 0.75; turn_decrease = 1.5 }
            .transitionTimed(sampleWait, SamplerState.GRAB_SAMPLE)
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW
            )

            .state(SamplerState.HOLD_SAMPLE)
            .onEnter { sampler.hold(); sampHeights = true; turn_decrease = 0.0 }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.SCORE_SAMPLE,
            )
            .transition(
                { currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8 },
                SamplerState.SPIT,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.SCORE_SAMPLE)
            .onEnter { sampler.score_sample(); sampHeights = true;
                // WARNING: this should probably be changed so that speed decrease only resets once lift lowers
                speed_decrease = 0.0
                turn_decrease = 0.0
            }
            .transitionTimed(sampleWait, SamplerState.STOW)
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SamplerState.STOW,
                { lift.setPreset(Lift.Preset.BOTTOM) }
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                SamplerState.STOW,
            )

            .state(SamplerState.PREPARE_TO_SCORE_SPECIMEN)
            .onEnter { sampler.prepare_to_score_specimen(); sampHeights = true; lift.setPreset(Lift.Preset.SPEC_HIGH_SCORE); }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8
                        && lift.getPreset() == Lift.Preset.SPEC_LOW },
                SamplerState.SCORE_SPECIMEN_LOW,
            )
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8
                        && lift.getPreset() == Lift.Preset.SPEC_HIGH },
                SamplerState.SCORE_SPECIMEN_HIGH,
            )
            .transition(
                { currentGamepad2.x && !previousGamepad2.x },
                // how would we eject an unwanted spec?
                SamplerState.STOW,
            )

            .state(SamplerState.SCORE_SPECIMEN_LOW)
            .onEnter { lift.setPreset(Lift.Preset.SPEC_LOW_SCORE) }
            // is after time always from beginning of state or does it count from previous after time? presumably the former
            .afterTime(1.0) { sampler.score_specimen() }
            // we might want the lift to go spec intake instead assuming we're doing chaining
            .transitionTimed(1.5, SamplerState.STOW) { lift.setPreset(Lift.Preset.BOTTOM) }

            .state(SamplerState.SCORE_SPECIMEN_HIGH)
            .onEnter() { lift.setPreset(Lift.Preset.SPEC_HIGH_SCORE) }
            .afterTime(1.0) { sampler.score_specimen() }
            .transitionTimed(1.5, SamplerState.STOW) { lift.setPreset(Lift.Preset.BOTTOM) }

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
            drivetrain.setEffort(
                -sign(gamepad1.left_stick_y.toDouble()) * gamepad1.left_stick_y.toDouble().pow(2) / (1 + speed_decrease),
                -sign(gamepad1.left_stick_x.toDouble()) * gamepad1.left_stick_x.toDouble().pow(2) / (1 + speed_decrease),
                -sign(gamepad1.right_stick_x.toDouble()) * gamepad1.right_stick_x.toDouble().pow(2) / (1.5 + turn_decrease),
            )

            // lift
            if (currentGamepad2.a && !previousGamepad2.a) { lift.setPreset(Lift.Preset.BOTTOM) ; speed_decrease = 0.0 }
            if (currentGamepad2.b && !previousGamepad2.b && sampHeights) { lift.setPreset(Lift.Preset.LOW) ; speed_decrease = 0.0 }
            if (currentGamepad2.y && !previousGamepad2.y && sampHeights) { lift.setPreset(Lift.Preset.HIGH) ; speed_decrease = 1.0 }

            if (currentGamepad2.b && !previousGamepad2.b && specHeights) { lift.setPreset(Lift.Preset.SPEC_LOW) ; speed_decrease = 0.0 }
            if (currentGamepad2.y && !previousGamepad2.y && specHeights) { lift.setPreset(Lift.Preset.SPEC_HIGH) ; speed_decrease = 1.0 }

            if (currentGamepad2.left_trigger > 0.8 &&
                currentGamepad2.right_trigger > 0.8 &&
                currentGamepad2.dpad_up &&
                !previousGamepad2.dpad_up
            ) {
                lift.setPreset(Lift.Preset.RAISE_HANG)
                speed_decrease = 1.5
            }
            if (currentGamepad2.left_trigger > 0.8 &&
                currentGamepad2.right_trigger > 0.8 &&
                currentGamepad2.dpad_down &&
                !previousGamepad2.dpad_down
            ) {
                lift.setPreset(Lift.Preset.PULL_HANG)
                speed_decrease = 1.5
            }
            if (currentGamepad2.start && !previousGamepad2.start) { manual = !manual }
            if (!manual) {
                lift.updateProfiled(lift.getHeight())
            } else {
                lift.setEffort(-gamepad2.left_stick_y + 0.2)
                if (gamepad2.dpad_up && -gamepad2.left_stick_y < -0.9) {
                    lift.resetEncoder()
                }
            }

            // sampler
            // TODO Add auto-retraction based on analog intake output and localization
            fsm.update()

            drivetrain.write()
            lift.write()
            sampler.write()

            telemetry.addLine("regular pathway - g1 left trigger")
            telemetry.addLine("alt pathway - g1 right trigger")
            telemetry.addLine("lift ctrls, samp & spec - g2 a, b, y")
            telemetry.addLine("reset to stow - g2 x")
//            telemetry.addLine("               ")
            telemetry.addLine("LIFT reset - g2 dpad up + left stick up")
            telemetry.addLine("               ")
            telemetry.addData("state", fsm.state)
//            telemetry.addData("turn", gamepad1.right_stick_x)
            telemetry.addData("height", lift.getHeight())
            telemetry.addData("intake velocity", intakeSpeed.speed)
            telemetry.addData("intake voltage", intakeSpeed.getVoltage)
//            telemetry.addData("total current", lift.leftCurrent() + lift.rightCurrent())
            telemetry.update()
        }
    }
}