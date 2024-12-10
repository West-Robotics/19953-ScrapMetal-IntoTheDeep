package org.firstinspires.ftc.teamcode.experimental

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.util.architecture.action
import com.scrapmetal.util.architecture.task
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.experimental.subsystem.Claw
import org.firstinspires.ftc.teamcode.experimental.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.experimental.subsystem.Lift

@TeleOp(name = "coroutines test")
class Teleop : LinearOpMode() {
    override fun runOpMode(): Unit = runBlocking {
        val robotState = RobotState(hardwareMap, gamepad1)
        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val claw = Claw(hardwareMap)
        val pipelines = Pipelines()
        // why is pipelines not green?
        val tasks = Tasks(this, robotState, pipelines, drivetrain, lift, claw)

        val hardwareAccess = task {
            while (opModeIsActive()) {
                println("I am in hardware loop")
                robotState.update()
                drivetrain.write()
                lift.write()
                claw.write()
                yield()
            }
        }

        // can you restart this? will it kill itself when one of its own actions causes the cancellation
        // of another? because the killing propagates to the parents
        val driverButtonInput = task {
            while (opModeIsActive()) {
                println("I am in button input loop")
                if (gamepad1.a) claw.setState(Claw.State.CLOSE)
                if (gamepad1.b) claw.setState(Claw.State.OPEN)
//                if (gamepad1.right_trigger > 0.5) tasks.runLiftTo(12.0).start()
//                if (gamepad1.left_trigger > 0.5) tasks.runLiftTo(0.0).start()
                if (gamepad1.x) tasks.scorePreload().start()
                yield()
            }
        }

        // readme name is incorrect
        waitForStart()
        println("started hardware: ${hardwareAccess.start()}")
        println("started buttons: ${driverButtonInput.start()}")
        tasks.driveByGamepad().start()
        launch { pipelines.drivetrain.start() }
        // launch { pipelines.lift.start() }
        // launch { pipelines.claw.start(); println("marker 4") }
        // println("marker 5")
    }
}