package org.firstinspires.ftc.teamcode.ninth.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "SampleTele")
class SampleTele: LinearOpMode() {

    // TODO: test reset, manual, charge bot, test positions of lift and intake, pow turn
    enum class SamplerState {
        SAMPLER_STOW,
        SAMPLER_EXTEND,
        SAMPLER_GRAB_SAMPLE,
        SAMPLER_SPIT,
        SAMPLER_HOLD,
        SAMPLER_SCORE_SAMPLE,
        SAMPLER_AUTOSCORE,
    }
    var samplerState = SamplerState.SAMPLER_STOW
    val samplerTimer = ElapsedTime()

    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)

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
                -sign(gamepad1.right_stick_x.toDouble()) * gamepad1.right_stick_x.toDouble().pow(2) / (2 + turn_decrease),
            )

            // lift
            if (currentGamepad2.a && !previousGamepad2.a) { lift.setPreset(Lift.Preset.BOTTOM) ; speed_decrease = 0.0 }
            if (currentGamepad2.b && !previousGamepad2.b) { lift.setPreset(Lift.Preset.LOW) ; speed_decrease = 1.0 }
            if (currentGamepad2.y && !previousGamepad2.y) { lift.setPreset(Lift.Preset.HIGH) ; speed_decrease = 1.5 }
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
            when (samplerState) {
                SamplerState.SAMPLER_STOW -> {
                    sampler.stow()
                    turn_decrease = 0.0
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 && lift.getPreset() == Lift.Preset.BOTTOM) {
                        samplerState = SamplerState.SAMPLER_EXTEND
                    }
                }
                SamplerState.SAMPLER_EXTEND -> {
                    sampler.extend()
                    turn_decrease = 1.0
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_GRAB_SAMPLE
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_GRAB_SAMPLE -> {
                    sampler.grab_sample()
                    turn_decrease = 1.0
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_HOLD
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                    if (currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_SPIT
                    }
                }
                SamplerState.SAMPLER_SPIT -> {
                    sampler.spit()
                    turn_decrease = 1.0
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_GRAB_SAMPLE
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_HOLD -> {
                    sampler.hold()
                    turn_decrease = 0.0
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_SCORE_SAMPLE
                    }
                    // if (currentGamepad1.right_trigger > 0.8) {
                    //     if (desiredPos == 0.0) {
                    //         samplerState = SamplerState.SAMPLER_AUTOSCORE
                    //     }
                    // }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_SCORE_SAMPLE -> {
                    sampler.score_sample()
                    turn_decrease = 0.0
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_AUTOSCORE -> {
                    samplerTimer.reset()
                    sampler.extend()
                    turn_decrease = 0.0
                    if (samplerTimer.seconds() >= 0.5) {
                        sampler.score_front()
                    }
                    if ((samplerTimer.seconds() > 1.0) or (currentGamepad2.x && !previousGamepad2.x)) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
            }

            drivetrain.write()
            lift.write()
            sampler.write()

//            telemetry.addLine("RETRACT - g2 left bumper")
            telemetry.addLine("EXTEND -> INTAKE -> HOLD -> SCORE (g1 left trigger)")
//            telemetry.addLine("SCORE sample - g1 right trig")
            telemetry.addLine("               ")
//            telemetry.addLine("EXTEND - g2 left trig")
            telemetry.addLine("SPIT - g2 left trig")
//            telemetry.addLine("RETRACT/HOLD sample - g2 right trig")
            telemetry.addLine("STOW - g2 x")
            telemetry.addLine("LIFT down - g2 a")
            telemetry.addLine("LIFT pos 1 - g2 b")
            telemetry.addLine("LIFT pos 2 - g2 y")
            telemetry.addLine("LIFT manual - g2 start")
            telemetry.addLine("LIFT reset - g2 dpad up + left stick up")
            telemetry.addLine("               ")
            telemetry.addData("state", samplerState)
            telemetry.addData("height", lift.getHeight())
            // TODO: remove current reads for comp for looptimes
            telemetry.addData("left current", lift.leftCurrent())
            telemetry.addData("right current", lift.rightCurrent())
            telemetry.addData("total current", lift.leftCurrent() + lift.rightCurrent())
            telemetry.update()
        }
    }
}