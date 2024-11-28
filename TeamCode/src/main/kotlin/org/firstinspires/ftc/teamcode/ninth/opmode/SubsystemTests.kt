package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.ninth.controlEffort
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

// TODO: migrate relevant teleop logic out of subsystem tests

@TeleOp(name = "SubsystemTests")
class SubsystemTests : LinearOpMode() {
    override fun runOpMode() {
        val previousGamepad1 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        var desiredPos = 0.0
//        val lift = Lift(hardwareMap)
        var manual = false
        val extensionAmount = 0.0
        val sampler = Sampler(hardwareMap)

        waitForStart()
        while (opModeIsActive()) {
            previousGamepad1.copy(gamepad1)

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
//                lift.setEffort(gamepad1.left_trigger - gamepad1.right_trigger.toDouble() + 0.1)
//            }
//
//            lift.write()

            // sampler
//            sampler.setExtensionAmount(extensionAmount)

//            if (gamepad1.a && !previousGamepad1.a) {
//                sampler.extend()
//            }
//            if (gamepad1.a && previousGamepad1.a) {
//                sampler.retract()
//            }
//            if (gamepad1.b && !previousGamepad1.b) {
//                sampler.grab()
//            }
//            if (gamepad1.b && previousGamepad1.b) {
//                sampler.stow()
//            }
//            if (gamepad1.x) {
//                sampler.score()
//            }

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
            telemetry.update()
        }
    }
}