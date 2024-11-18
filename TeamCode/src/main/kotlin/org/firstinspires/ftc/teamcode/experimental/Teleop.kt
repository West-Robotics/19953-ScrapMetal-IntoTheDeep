package org.firstinspires.ftc.teamcode.experimental

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.util.architecture.action
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.experimental.subsystem.Claw
import org.firstinspires.ftc.teamcode.experimental.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.experimental.subsystem.Lift

@TeleOp(name = "coroutines test")
class Teleop : LinearOpMode() {
    override fun runOpMode(): Unit = runBlocking {
        val robotState = RobotState(hardwareMap, gamepad1)
        val pipelines = Pipelines()
        val inputs = Inputs()
        val tasks = Tasks(robotState, pipelines, inputs)
        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val claw = Claw(hardwareMap)

        val hardwareAccess = action {
            while (opModeIsActive()) {
                robotState.update()
                // uh oh, will this hang if there are no more inputs? maybe switch inputs to StateFlow
                drivetrain.drive(inputs.drivetrain.receive())
                lift.setEffort(inputs.lift.receive())
                claw.setState(inputs.claw.receive())
            }
        }

        // can you restart this? will it kill itself when one of its own actions causes the cancellation
        // of another? because the killing propagates to the parents
        val driverButtonInput = action {
            while (opModeIsActive()) {
                if (gamepad1.a) inputs.claw.send(Claw.State.OPEN)
                if (gamepad1.b) inputs.claw.send(Claw.State.GRAB)
                if (gamepad1.right_trigger > 0.5) tasks.runLiftTo(12.0).start()
                if (gamepad1.left_trigger > 0.5) tasks.runLiftTo(0.0).start()
                if (gamepad1.x) tasks.scorePreload().start()
            }
        }

        // readme name is incorrect
        waitForStart()
        hardwareAccess.start()
        driverButtonInput.start()
        tasks.driveByGamepad().start()
    }
}