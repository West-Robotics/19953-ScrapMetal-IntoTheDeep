package org.firstinspires.ftc.teamcode.ninth.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.scrapmetal.util.control.pathing.drawRobot
import com.scrapmetal.util.control.pathing.drawTrail
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@TeleOp(name = "Drivetrain Test")
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap)

        val dashboard = FtcDashboard.getInstance()
        waitForStart()
        while (opModeIsActive()) {
            drivetrain.setEffort(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            )
            drivetrain.write()

            val packet = TelemetryPacket()
            packet.fieldOverlay()
                .drawRobot(drivetrain.getPose())
                .drawTrail(drivetrain.getPose().position)
            dashboard.sendTelemetryPacket(packet)
            dashboard.telemetry.update()
        }
    }
}
