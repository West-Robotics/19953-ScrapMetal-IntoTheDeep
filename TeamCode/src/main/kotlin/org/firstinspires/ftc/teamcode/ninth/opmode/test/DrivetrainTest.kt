package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.pathing.drawRobot
import com.scrapmetal.util.control.pathing.drawTrail
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain

@TeleOp(name = "Drivetrain Test")
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap)
        val poseA = Pose2d(0.0, 0.0, 0.0)
        val poseB = Pose2d(48.0, 24.0, 180.0)
        var currentPose = poseA

        val dashboard = FtcDashboard.getInstance()
        waitForStart()
        drivetrain.setPose(0.0, 0.0, 0.0)
        while (opModeIsActive()) {
            val (pos, _) = drivetrain.getPoseAndVelo()

            drivetrain.setEffort(
                when {
                    gamepad1.a -> { currentPose = poseA; drivetrain.getPDEffort(poseA) }
                    gamepad1.b -> { currentPose = poseB; drivetrain.getPDEffort(poseB) }
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
            dashboard.telemetry.addData("x", pos.position.x)
            dashboard.telemetry.addData("y", pos.position.y)
            dashboard.telemetry.addData("heading", pos.heading.theta)
            dashboard.telemetry.addData("x error", currentPose.position.x - pos.position.x)
            dashboard.telemetry.addData("y error", currentPose.position.y - pos.position.y)
            dashboard.telemetry.addData("heading error", currentPose.heading.theta - pos.heading.theta)
            dashboard.telemetry.update()
        }
    }
}
