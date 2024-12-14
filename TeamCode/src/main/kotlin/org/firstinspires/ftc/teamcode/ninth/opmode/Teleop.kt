package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "NinthTele")
class Teleop: LinearOpMode() {

    // TODO: test reset, manual, charge bot, test positions of lift and intake, pow turn
    enum class SamplerState {
        SAMPLER_STOW,
        SAMPLER_EXTEND,
        SAMPLER_GRAB,
        SAMPLER_SPIT,
        SAMPLER_HOLD,
        SAMPLER_SCORE,
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

//        val liftHeight = lift.getHeight()
        var desiredPos = 0.0
        var manual = false

        waitForStart()
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)

            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            // drive
            drivetrain.setVelocity(
                -sign(gamepad1.left_stick_y.toDouble()) * gamepad1.left_stick_y.toDouble().pow(2),
                -sign(gamepad1.left_stick_x.toDouble()) * gamepad1.left_stick_x.toDouble().pow(2),
                -sign(gamepad1.right_stick_x.toDouble()) * gamepad1.right_stick_x.toDouble().pow(2) / 2,
            )
            drivetrain.write()

            // lift
            // TODO: implement hardstop for lift

            if (currentGamepad2.a && !previousGamepad2.a) {
                desiredPos = 0.0
            }
            if (currentGamepad2.b && !previousGamepad2.b) {
                desiredPos = 25.75 - 7.5
            }
            if (currentGamepad2.y && !previousGamepad2.y) {
                desiredPos = 43.0 - 7.5
            }

            if (currentGamepad2.start && !previousGamepad2.start) {
                manual = !manual
            }

            if (!manual) {
                lift.runToPos(desiredPos, lift.getHeight())
            } else {
                lift.setEffort(-gamepad2.left_stick_y + 0.2)
                if (gamepad2.dpad_up && -gamepad2.left_stick_y < -0.9) {
                    lift.resetEncoder()
                }
            }

            lift.write()

            // sampler
            when (samplerState) {
                SamplerState.SAMPLER_STOW -> {
                    sampler.stow()
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 && desiredPos == 0.0) {
                        samplerState = SamplerState.SAMPLER_EXTEND
                    }
                }
                SamplerState.SAMPLER_EXTEND -> {
                    sampler.extend()
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_GRAB
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_GRAB -> {
                    sampler.grab()
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
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_GRAB
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_HOLD -> {
                    sampler.hold()
                    if (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8) {
                        samplerState = SamplerState.SAMPLER_SCORE
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
                SamplerState.SAMPLER_SCORE -> {
                    sampler.score()
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
                    if (samplerTimer.seconds() >= 0.5) {
                        sampler.score_front()
                    }
                    if ((samplerTimer.seconds() > 1.0) or (currentGamepad2.x && !previousGamepad2.x)) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
            }

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
            telemetry.update()
        }
    }
}