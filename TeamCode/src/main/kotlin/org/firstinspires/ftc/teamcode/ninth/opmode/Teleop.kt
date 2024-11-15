package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import kotlin.math.E
import kotlin.time.Duration.Companion.seconds

@TeleOp(name = "NinthTele")
class Teleop: LinearOpMode() {
    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)

        val l1BumperTime = ElapsedTime()
        var l1BumperState = 0.0
        val l2BumperTime = ElapsedTime()
        var l2BumperState = 0.0
        val rBumperTime = ElapsedTime()
        var rBumperState = 0.0

        waitForStart()
        val timeSinceInit = ElapsedTime()

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
            // TODO: implement hardstop for lift
            val desiredPos = when {
                (gamepad2.a && !previousGamepad2.a) -> 24.0
                (gamepad2.b && !previousGamepad2.b) -> 36.0
                else -> 0.0
            }

            val liftHeight = lift.getHeight()
            val liftK = 0.0

//            val liftPower = 0.7
//            if ((gamepad2.a) or (gamepad2.b)) {
//                lift.setPower(lift.runToPreset(desiredPos, liftHeight, liftPower))
//                sampler.score()
//            } else {
//                lift.setPower((-gamepad2.right_stick_y).toDouble())
//            }

            if ((gamepad2.a && previousGamepad2.a) or (gamepad2.b && previousGamepad2.b))
                lift.setPower(lift.controlEffort(desiredPos, liftHeight, liftK))

            // sampler
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