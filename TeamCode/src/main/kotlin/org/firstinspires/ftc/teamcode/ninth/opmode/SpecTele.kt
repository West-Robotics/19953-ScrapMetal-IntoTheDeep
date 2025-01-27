package org.firstinspires.ftc.teamcode.ninth.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.hardware.SMAnalog
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
    var samplerState = SamplerState.STOW
    val samplerTimer = ElapsedTime()
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
        val intakeSpeed = SMAnalog(hardwareMap, "analogOutput")

        val intake_stalled = 5.0

        var desiredPos = Lift.Preset.LOW
        var manual = false
//        lift.setPreset(Lift.Preset.LOW)
        lift.setPreset(Lift.Preset.BOTTOM)

        var speed_decrease = 0.0
        var turn_decrease = 0.0

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
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
            when (samplerState) {
                SamplerState.STOW -> {
                    sampler.stow()
                    speed_decrease = 0.0
                    turn_decrease = 0.0
                    sampHeights = false
                    specHeights = false
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 && lift.getPreset() == Lift.Preset.BOTTOM) {
                        samplerState = SamplerState.EXTEND
                    }
                    if (currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8 && lift.getPreset() == Lift.Preset.BOTTOM) {
                        samplerState = SamplerState.GRAB_SPECIMEN
                    }
                }
                SamplerState.EXTEND -> {
                    sampler.extend()
                    speed_decrease = 0.0
                    turn_decrease = 1.5
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.GRAB_SAMPLE
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                SamplerState.GRAB_SAMPLE -> {
                    sampler.grab_sample()
                    speed_decrease = 0.75
                    turn_decrease = 1.5
                    if (intakeSpeed.velocity < intake_stalled) {
                        samplerState = SamplerState.HOLD_SAMPLE
                    }
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        speed_decrease = 0.0
                        samplerState = SamplerState.HOLD_SAMPLE
                    }
                    if (currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8) {
                        samplerState = SamplerState.GRAB_SAMPLE_SIDE
                    }
                    if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                        samplerState = SamplerState.SPIT
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                SamplerState.GRAB_SAMPLE_SIDE -> {
                    sampler.grab_sample_side()
                    if (currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8) {
                        samplerState = SamplerState.GRAB_SAMPLE
                    }
                    if (intakeSpeed.velocity < intake_stalled) {
                        samplerState = SamplerState.HOLD_SAMPLE
                    }
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        speed_decrease = 0.0
                        samplerState = SamplerState.HOLD_SAMPLE
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                SamplerState.GRAB_SPECIMEN -> {
                    sampler.grab_specimen()
                    lift.setPreset(Lift.Preset.SPEC_INTAKE)
                    if (intakeSpeed.velocity < intake_stalled) {
                      samplerState = SamplerState.HOLD_SPECIMEN
                    }
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 && lift.getPreset() == Lift.Preset.BOTTOM) {
                        samplerState = SamplerState.HOLD_SPECIMEN
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                SamplerState.HOLD_SPECIMEN -> {
                    lift.setPreset(Lift.Preset.SPEC_INTAKE)
                    sampler.hold()
                    specHeights = true
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.PREPARE_TO_SCORE_SPECIMEN
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                SamplerState.SPIT -> {
                    speed_decrease = 0.75
                    turn_decrease = 1.5
                    samplerTimer.reset()
                    sampler.spit()
                    if (samplerTimer.seconds() >= sampleWait) {
                        samplerState = SamplerState.GRAB_SAMPLE
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                SamplerState.HOLD_SAMPLE -> {
                    sampler.hold()
                    sampHeights = true
                    turn_decrease = 0.0
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SCORE_SAMPLE
                    }
                    if (currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8) {
                        samplerState = SamplerState.SPIT
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                SamplerState.SCORE_SAMPLE -> {
                    sampler.score_sample()
                    sampHeights = true
                    samplerTimer.reset()
                    if (samplerTimer.seconds() >= sampleWait) {
                        samplerState = SamplerState.STOW
                    }
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        lift.setPreset(Lift.Preset.BOTTOM)
                        samplerState = SamplerState.STOW
                    }
                    speed_decrease = 0.0
                    turn_decrease = 0.0
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                // Not sure if this state is necessary, iuli put it in the control scheme
                SamplerState.PREPARE_TO_SCORE_SPECIMEN -> {
                    sampHeights = true
                    sampler.prepare_to_score_specimen()
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        if (lift.getPreset() == Lift.Preset.LOW) {
                            samplerState = SamplerState.SCORE_SPECIMEN_LOW
                        }
                        if (lift.getPreset() == Lift.Preset.HIGH) {
                            samplerState = SamplerState.SCORE_SPECIMEN_HIGH
                        }
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.STOW
                    }
                }
                SamplerState.SCORE_SPECIMEN_LOW -> {
                    samplerTimer.reset()
                    if (samplerTimer.seconds() > 1.5) {
                        lift.setPreset(Lift.Preset.SPEC_LOW_SCORE)
                    }
                    if (samplerTimer.seconds() > 2.0) {
                        sampler.score_specimen()
                    }
                    if (samplerTimer.seconds() > 2.5) {
                        samplerState = SamplerState.STOW
                        lift.setPreset(Lift.Preset.BOTTOM)
                    }
                }
                SamplerState.SCORE_SPECIMEN_HIGH -> {
                    samplerTimer.reset()
                    if (samplerTimer.seconds() > 1.5) {
                        lift.setPreset(Lift.Preset.SPEC_HIGH_SCORE)
                    }
                    if (samplerTimer.seconds() > 2.0) {
                        sampler.score_specimen()
                    }
                    if (samplerTimer.seconds() > 2.5) {
                        samplerState = SamplerState.STOW
                        lift.setPreset(Lift.Preset.BOTTOM)
                    }
                }
            }

            drivetrain.write()
            lift.write()
            sampler.write()

            //TODO Update Telemetry for new control system
//            telemetry.addLine("RETRACT - g2 left bumper")
//            telemetry.addLine("EXTEND -> INTAKE -> HOLD -> SCORE (g1 left trigger)")
//            telemetry.addLine("SCORE sample - g1 right trig")
//            telemetry.addLine("               ")
//            telemetry.addLine("EXTEND - g2 left trig")
//            telemetry.addLine("SPIT - g2 left trig")
//            telemetry.addLine("RETRACT/HOLD sample - g2 right trig")
//            telemetry.addLine("STOW - g2 x")
//            telemetry.addLine("LIFT down - g2 a")
//            telemetry.addLine("LIFT pos 1 - g2 b")
//            telemetry.addLine("LIFT pos 2 - g2 y")
//            telemetry.addLine("LIFT manual - g2 start")
            telemetry.addLine("LIFT reset - g2 dpad up + left stick up")
            telemetry.addLine("               ")
            telemetry.addData("state", samplerState)
            telemetry.addData("turn", gamepad1.right_stick_x)
            telemetry.addData("height", lift.getHeight())
            // TODO: remove current reads for comp for looptimes
            telemetry.addData("left current", lift.leftCurrent())
            telemetry.addData("right current", lift.rightCurrent())
            telemetry.addData("total current", lift.leftCurrent() + lift.rightCurrent())
            telemetry.update()
        }
    }
}