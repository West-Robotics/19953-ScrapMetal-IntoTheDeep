package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift

@TeleOp(name = "NinthTele")
class Teleop: LinearOpMode() {
    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)

        waitForStart()
        while (opModeIsActive()) {
            previousGamepad1.copy(gamepad1)
            previousGamepad2.copy(gamepad2)

            // drive
            drivetrain.setVelocity(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            )

            // lift
            val desiredPos = when {
                (gamepad2.a) -> 24.0
                (gamepad2.b) -> 36.0
                else -> 0.0
            }

            val liftHeight = lift.getHeight()
            val liftPower = 0.7

            if ((gamepad2.a) or (gamepad2.b)) {
                lift.setPower(lift.runToPreset(desiredPos, liftHeight, liftPower))
            } else {
                lift.setPower((-gamepad2.right_stick_y).toDouble())
            }

            // sampler
        }
    }
}