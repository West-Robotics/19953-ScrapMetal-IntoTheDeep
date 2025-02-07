package org.firstinspires.ftc.teamcode.ninth.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "0+1")
class SampAuto: LinearOpMode() {
    override fun runOpMode() {

        // val drivetrain = Drivetrain(hardwareMap)
        // val lift = Lift(hardwareMap)
        // val sampler = Sampler(hardwareMap)

        // waitForStart()
        // val timer = ElapsedTime()

        // while (opModeIsActive()) {
        //     if (timer.seconds() <= 4.0) {
        //         drivetrain.setEffort(0.0, 0.0, 0.0)
        //         lift.runToPos(43.0 - 7.5, lift.getHeight())
        //         sampler.stow() // stow on purpose, not hold
        //     }
        //     if (4.0 < timer.seconds() && timer.seconds() <= 6.25) {
        //         drivetrain.setEffort(0.2, 0.0, 0.0)
        //         lift.runToPos(43.0 - 7.5, lift.getHeight())
        //         sampler.stow() // stow on purpose, not hold
        //     }
        //     if (6.25 < timer.seconds() && timer.seconds() <= 8.0) {
        //         drivetrain.setEffort(0.0, 0.0, 0.0)
        //         lift.runToPos(43.0 - 7.5, lift.getHeight())
        //         sampler.score()
        //     }
        //     if (8.0 < timer.seconds() && timer.seconds() <= 9.0) {
        //         drivetrain.setEffort(-0.2, 0.0, 0.0)
        //         lift.runToPos(43.0 - 7.5, lift.getHeight())
        //         sampler.stow()
        //     }
        //     if (9.0 < timer.seconds() && timer.seconds() <= 12.8) {
        //         drivetrain.setEffort(0.0, -0.3, 0.0)
        //         lift.runToPos(0.0, lift.getHeight())
        //         sampler.stow()
        //     }
        //     if (12.8 < timer.seconds() && timer.seconds() <= 16.0) {
        //         drivetrain.setEffort(0.0, 0.0, 0.0)
        //         lift.runToPos(25.75 - 7.5, lift.getHeight())
        //         sampler.stow()
        //     }
        //     if (16.0 < timer.seconds() && timer.seconds() <= 19.0) {
        //         drivetrain.setEffort(-0.2, 0.0, 0.0)
        //         lift.runToPos(25.75 - 7.5, lift.getHeight())
        //         sampler.extend()
        //     }
        //     if (19.0 < timer.seconds()) {
        //         drivetrain.setEffort(0.0, 0.0, 0.0)
        //         lift.setEffort(0.0)
        //         sampler.extend()
        //     }

        //     drivetrain.write()
        //     lift.write()
        // }
    }
}
