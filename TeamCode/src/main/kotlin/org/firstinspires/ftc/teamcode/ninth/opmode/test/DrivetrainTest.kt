package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.pathing.drawRobot
import com.scrapmetal.util.control.pathing.drawTrail
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain

@TeleOp(name = "Drivetrain Test")
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() = runBlocking {
        val drivetrain = Drivetrain(hardwareMap)
        val poseA = Pose2d(0.0, 0.0, 0.0)
        val poseB = Pose2d(24.0, 24.0, 90.0)

        val dashboard = FtcDashboard.getInstance()
        waitForStart()
        drivetrain.setPose(0.0, 0.0, 0.0)
        val timer = ElapsedTime()
        drivetrain.beginPinpoint(this)
        while (opModeIsActive()) {
            val (pos, _) = drivetrain.getPoseAndVelo(debug = dashboard.telemetry)

            drivetrain.setEffort(
                when {
                    gamepad1.a -> { drivetrain.getPDEffort(poseA, debug = dashboard.telemetry) }
                    gamepad1.b -> { drivetrain.getPDEffort(poseB, debug = dashboard.telemetry) }
                    else -> Pose2d(
                        -gamepad1.left_stick_y.toDouble(),
                        -gamepad1.left_stick_x.toDouble(),
                        -gamepad1.right_stick_x.toDouble(),
                    )
                }
            )
            drivetrain.write()

            val packet = TelemetryPacket()
            packet.fieldOverlay()
                .drawRobot(pos)
                .drawTrail(pos.position)
            dashboard.sendTelemetryPacket(packet)
            dashboard.telemetry.addData("loop time", timer.milliseconds())
            dashboard.telemetry.addData("gp x", -gamepad1.left_stick_y)
            dashboard.telemetry.addData("gp y", -gamepad1.left_stick_x)
            dashboard.telemetry.addData("gp turn", -gamepad1.right_stick_x)
            timer.reset()
            dashboard.telemetry.update()
        }
    }
}
