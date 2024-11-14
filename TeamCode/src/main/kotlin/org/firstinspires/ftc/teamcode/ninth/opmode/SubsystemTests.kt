package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@TeleOp(name = "SubsystemTests")
class SubsystemTests : LinearOpMode() {
    override fun runOpMode() {
        val previousGamepad1 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)

        waitForStart()
        while (opModeIsActive()) {
            previousGamepad1.copy(gamepad1)

            // drive
            drivetrain.setVelocity(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            )

            // lift
            val liftPower = 0.7
            if (gamepad1.left_trigger.toDouble() > 0.9) {
                lift.setPower(liftPower)
            }
            if (gamepad1.right_trigger.toDouble() > 0.9) {
                lift.setPower(-liftPower)
            }

            // intake
            if (gamepad1.a && !previousGamepad1.a) {
                sampler.extend()
            }
            if (gamepad1.a && previousGamepad1.a) {
                sampler.retract()
            }
            if (gamepad1.b && !previousGamepad1.b) {
                sampler.grab()
            }
            if (gamepad1.b && previousGamepad1.b) {
                sampler.stow()
            }
            if (gamepad1.x) {
                sampler.score()
            }

            val liftHeight = lift.getHeight()
            telemetry.addLine("Raise lift - hold left trigger")
            telemetry.addLine("Lower lift - hold right trigger")
            telemetry.addLine("Extend sampler - press a")
            telemetry.addLine("Retract sampler - press a")
            telemetry.addLine("Intake sample - press b")
            telemetry.addLine("Stow intake - press b")
            telemetry.addLine("Score - hold x")

            telemetry.addData("lift height", liftHeight)
            telemetry.update()
        }
    }
}