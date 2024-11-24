package org.firstinspires.ftc.teamcode.ninth.opmode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@TeleOp(name = "SubsystemTests")
class SubsystemTests : LinearOpMode() {
    @Config
    companion object {
        @JvmField var amount = 0.0
    }
    override fun runOpMode() {
        val previousGamepad1 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        var desiredPos = 0.0
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
            drivetrain.writeDtEffort()

            // lift
            lift.setPower(gamepad1.left_trigger-gamepad1.right_trigger.toDouble())
//            val liftPower = 0.7
//            if (gamepad1.left_trigger.toDouble() > 0.9) {
//                lift.setPower(liftPower)
//            } else if (gamepad1.right_trigger.toDouble() > 0.9) {
//                lift.setPower(-liftPower)
//            } else {
//                lift.setPower(0.0)
//            }
            lift.writeLiftEffort()

            if (gamepad1.a && previousGamepad1.a) {
                desiredPos = 0.0
            }
            if (gamepad1.b && previousGamepad1.b) {
                desiredPos = 24.0
            }
            if (gamepad1.y && previousGamepad1.y) {
                desiredPos = 36.0
            }

            val liftHeight = lift.getHeight()
//            val p = 1.0
//            val f = 0.0
//
//            if ((gamepad2.a && previousGamepad1.a) or (gamepad2.b && previousGamepad1.b)) {
//                lift.setPower(lift.controlEffort(desiredPos, liftHeight, p, f))
//            } else {
//                lift.setPower(gamepad2.left_stick_y.toDouble())
//            }
//
//            // sampler
                sampler.setExtensionAmount(amount)
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

            telemetry.addData("lift height: ", liftHeight)
            telemetry.addData("Desired position: ", desiredPos)
            telemetry.update()
        }
    }
}