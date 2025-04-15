package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.util.hardware.SMGamepad
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.State.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Roll.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Color.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift.Preset.*
import kotlin.math.abs

@TeleOp(name="Gamepad Test")
class GamepadTest : LinearOpMode() {
    override fun runOpMode() {
        val driver = SMGamepad(gamepad1)
        val operator = SMGamepad(gamepad2)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        while (opModeIsActive()) {
            driver.update()
            operator.update()

            telemetry.addData("d lsy", -driver.lsy.sq)
            telemetry.addData("d lsx", -driver.lsx.sq)
            telemetry.addData("d rsx", -driver.rsx.sq)

            telemetry.update()
        }
    }
}
