package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift

@Autonomous(name = "Spec-side park")
class SpecParkAuto: LinearOpMode() {
    override fun runOpMode() {

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)

        waitForStart()
        val timer = ElapsedTime()

        while (opModeIsActive()) {
            if (timer.seconds() <= 3.0) {
                drivetrain.setEffort(0.3, 0.0, 0.0)
                sampler.stow() // stow on purpose, not hold
            }
            if (3.0 < timer.seconds()) {
                drivetrain.setEffort(0.0, 0.0, 0.0)
                sampler.stow() // stow on purpose, not hold
            }
            drivetrain.write()
            lift.write()
            sampler.write()
        }
    }
}