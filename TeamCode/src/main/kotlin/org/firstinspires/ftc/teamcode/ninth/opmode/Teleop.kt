package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ninth.controlEffort
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift

@TeleOp(name = "NinthTele")
class Teleop: LinearOpMode() {

    enum class SamplerState {
        SAMPLER_STOW,
        SAMPLER_EXTEND,
        SAMPLER_GRAB,
        SAMPLER_HOLD,
        SAMPLER_SCORE,
    }
    var samplerState = SamplerState.SAMPLER_STOW

    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)

        val liftHeight = lift.getHeight()
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
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            )

            // lift
            // TODO: implement hardstop for lift

            if (currentGamepad1.a && !previousGamepad1.a) {
                desiredPos = 0.0
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                desiredPos = 25.75 - 9
            }
            if (currentGamepad1.y && !previousGamepad1.y) {
                desiredPos = 43.0 - 9
            }

            if (currentGamepad1.start && !previousGamepad1.start) {
                manual = !manual
            }

            if (!manual) {
                lift.runToPos(desiredPos, liftHeight)
            } else {
                lift.setEffort(gamepad1.left_trigger - gamepad1.right_trigger.toDouble() + 0.1)
            }

            // sampler
            when (samplerState) {
                SamplerState.SAMPLER_STOW -> {
                    sampler.stow()
                    if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                        samplerState = SamplerState.SAMPLER_EXTEND
                    }
                }
                SamplerState.SAMPLER_EXTEND -> {
                    sampler.extend()
                    if (currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
                        samplerState = SamplerState.SAMPLER_GRAB
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_GRAB -> {
                    sampler.grab()
                    if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                        samplerState = SamplerState.SAMPLER_HOLD
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_HOLD -> {
                    sampler.hold()
                    if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                        samplerState = SamplerState.SAMPLER_SCORE
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
                SamplerState.SAMPLER_SCORE -> {
                    sampler.score()
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        samplerState = SamplerState.SAMPLER_STOW
                    }
                }
            }

            telemetry.addLine("EXTEND - g2 left bumper")
            telemetry.addLine("RETRACT - g2 left bumper")
            telemetry.addLine("INTAKE sample - g1 left bumper")
            telemetry.addLine("SCORE sample - g1 right bumper")
            telemetry.addLine("               ")
            telemetry.addLine("LIFT down - g2 a")
            telemetry.addLine("lift pos 1 - g2 b")
            telemetry.addLine("lift pos 2 - g2 y")
            telemetry.addLine("stow intake - g2 x")
            telemetry.addLine("               ")
            telemetry.addData("state", samplerState)
            telemetry.update()

        }
    }
}