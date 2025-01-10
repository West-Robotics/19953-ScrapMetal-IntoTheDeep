package org.firstinspires.ftc.teamcode.ninth.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.scrapmetal.util.control.pathing.drawRobot
import com.scrapmetal.util.control.pathing.drawTrail
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@TeleOp(name = "Drivetrain Test")
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap)

        val dashboard = FtcDashboard.getInstance()
        waitForStart()
        drivetrain.setPose(0.0, 0.0, 0.0)
        while (opModeIsActive()) {
            drivetrain.setEffort(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            )
            drivetrain.write()
            val pos = drivetrain.getPose()

            val packet = TelemetryPacket()
            packet.fieldOverlay()
                .drawRobot(pos)
                .drawTrail(drivetrain.getPose().position)
            dashboard.sendTelemetryPacket(packet)
            dashboard.telemetry.addData("x", drivetrain.getPose().position.x)
            dashboard.telemetry.addData("y", drivetrain.getPose().position.y)
            dashboard.telemetry.addData("heading", drivetrain.getPose().heading.theta)
            dashboard.telemetry.update()
            // telemetry.addData("x", drivetrain.getPose().position.x)
            // telemetry.addData("y", pos.getY(DistanceUnit.INCH))
            // telemetry.addData("heading", pos.getHeading(AngleUnit.DEGREES))
            // telemetry.update()
        }
    }
}
