package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.scrapmetal.util.hardware.SMGamepad
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@TeleOp(name = "Manual Ascent")
class ManualAscent : LinearOpMode() {
    override fun runOpMode() {
        val driver = SMGamepad(gamepad1)
        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage).coerceAtLeast(1.0), drivetrain)
        val sampler = Sampler(hardwareMap)
        sampler.state = Sampler.State.STOW
        var manual = true
        var engaged = false
        waitForStart()
        while (opModeIsActive()) {
            driver.update()
            lift.read()

            if (!engaged) {
                lift.effort = -driver.rsy.pos
            }
            if (!engaged && driver.lt.rising && driver.rt.rising && driver.lb.rising && driver.rb.rising) {
                lift.pto1ENGAGE()
                engaged = true
            }
            if (engaged) {
                lift.pto2MANUALCLIMB(-driver.rsy.pos)
            }
            sampler.updateProfiled()

            drivetrain.write()
            lift.write()
            sampler.write()

            telemetry.update()
        }
    }
}
