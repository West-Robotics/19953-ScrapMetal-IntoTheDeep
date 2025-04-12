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

@TeleOp(name="Manual Lift")
open class ManualLift : LinearOpMode() {
    override fun runOpMode() {
        val driver = SMGamepad(gamepad1)
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage).coerceAtLeast(1.0))
        lift.preset = BOTTOM

        var manual = false

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        while (opModeIsActive()) {
            driver.update()

            // lift
            lift.read()
            if (driver.guide.rising) {
                manual = !manual
            }
            if (!manual) {
                if (driver.a.rising) {
                    lift.preset = BOTTOM
                }
                if (driver.b.rising) { lift.preset = SAMP_LOW }
                if (driver.y.rising) { lift.preset = SAMP_HIGH }

                lift.updateProfiled(lift.height, debug = telemetry)
            } else {
                lift.effort = -driver.lsy.pos + lift.ff
            }
            lift.write()

            telemetry.update()
        }
    }
}
