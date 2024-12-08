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

        val l1BumperTime = ElapsedTime()
        var l1BumperState = 0.0
        val l2BumperTime = ElapsedTime()
        var l2BumperState = 0.0
        val rBumperTime = ElapsedTime()
        var rBumperState = 0.0

        waitForStart()
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)

            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad1)

            // drive
            drivetrain.setVelocity(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            )

            // lift
            // TODO: implement hardstop for lift

            if (gamepad1.a && previousGamepad1.a) {
                desiredPos = 0.0
            }
            if (gamepad1.b && previousGamepad1.b) {
                desiredPos = 25.75 - 9
            }
            if (gamepad1.y && previousGamepad1.y) {
                desiredPos = 43.0 - 9
            }

            if (gamepad1.start && !previousGamepad1.start) {
                manual = !manual
            }

            if (!manual) {
                lift.runToPos(desiredPos, liftHeight)
            } else {
                lift.setEffort(gamepad1.left_trigger - gamepad1.right_trigger.toDouble() + 0.1)
            }

            // sampler
            // TODO: impliment finite state machines
            if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
                sampler.extend()
                l1BumperTime.reset()
                l1BumperState = 1.0
            }
            if ((l1BumperTime.seconds() == 1.0) and (l1BumperState == 1.0)) {
                sampler.grab()
                l1BumperState = 0.0
            }

            if (gamepad1.left_bumper && previousGamepad1.left_bumper) {
                sampler.retract()
                l2BumperTime.reset()
                l2BumperState = 1.0
            }
            if ((l2BumperTime.seconds() == 1.0) and (l2BumperState == 1.0)) {
                sampler.stow()
                l2BumperState = 0.0
            }

            if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                sampler.release()
                rBumperTime.reset()
                rBumperState = 1.0
            }
            if ((rBumperTime.seconds() == 1.0) and (rBumperState == 1.0)) {
                sampler.stow()
                rBumperState = 0.0
            }
        }
    }
}