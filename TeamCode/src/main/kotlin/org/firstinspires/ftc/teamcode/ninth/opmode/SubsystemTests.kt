package org.firstinspires.ftc.teamcode.ninth.opmode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

// TODO: migrate relevant teleop logic out of subsystem tests

@TeleOp(name = "SubsystemTests")
//@Config
class SubsystemTests : LinearOpMode() {
//    @JvmField var stow = 0.0
//    @JvmField var score = 0.0

    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val currentGamepad1 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        var desiredPos = 0.0
        val lift = Lift(hardwareMap)
        var manual = false
//        var extensionAmount = 0.43
        val sampler = Sampler(hardwareMap)
        var intakeToggle = false
        var extensionToggle = false
//        var score = 0.43

        waitForStart()
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            currentGamepad1.copy(gamepad1)

//            if (gamepad1.a && previousGamepad1.a) {
//                desiredPos = 0.0
//            }
//            if (gamepad1.b && previousGamepad1.b) {
//                desiredPos = 25.75 - 9
//            }
//            if (gamepad1.y && previousGamepad1.y) {
//                desiredPos = 43.0 - 9
//            }
//
//            if (gamepad1.start && !previousGamepad1.start) {
//                manual = !manual
//            }

            // drive
//            drivetrain.setVelocity(
//                -gamepad1.left_stick_y.toDouble(),
//                -gamepad1.left_stick_x.toDouble(),
//                -gamepad1.right_stick_x.toDouble(),
//            )
//            drivetrain.write()

            // lift
//            val liftHeight = lift.getHeight()

//            if (!manual) {
//                lift.runToPos(desiredPos, liftHeight)
//            } else {
              lift.setEffort(gamepad1.left_trigger - gamepad1.right_trigger.toDouble() + 0.15)
//            }
//
//            lift.write()

            // sampler

//            if (currentGamepad1.a && !previousGamepad1.a) {
//                if (extensionToggle) {
//                    sampler.extend()
//                } else {
//                    sampler.retract()
//                }
//                extensionToggle = !extensionToggle
//            }
//
//            if (currentGamepad1.b && !previousGamepad1.b) {
//                if (intakeToggle) {
//                    sampler.grab()
//                } else {
//                    sampler.stow()
//                }
//                intakeToggle = !intakeToggle
//            }
//
//            if (currentGamepad1.x && !previousGamepad1.x) {
//                sampler.score()
//            }
//            if (currentGamepad1.y && !previousGamepad1.y) {
//                extensionAmount = 0.0
//            }
//            if (currentGamepad1.y && previousGamepad1.y) {
//                extensionAmount = 0.68
//            }

//            if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
//                score += 0.01
//            }
//            if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
//                score -= 0.01
//            }

            lift.write()

            telemetry.addLine("Raise lift - hold left trigger")
            telemetry.addLine("Lower lift - hold right trigger")

            telemetry.addLine("Extend sampler - press a")
            telemetry.addLine("Retract sampler - press a")
            telemetry.addLine("Intake sample - press b")
            telemetry.addLine("Stow intake - press b")
            telemetry.addLine("Score - hold x")
            telemetry.addLine("           ")
//            telemetry.addData("Lift height", liftHeight)
//            telemetry.addData("Control Effort", lift.getEffort())
//            telemetry.addData("Desired position", desiredPos)
//            telemetry.addData("Scoring pos", score)
//            telemetry.addData("Scoring pos", score)
//            telemetry.addData("extension amount", extensionAmount)

            telemetry.update()
        }
    }
}