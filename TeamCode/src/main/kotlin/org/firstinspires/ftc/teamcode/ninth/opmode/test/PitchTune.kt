package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@TeleOp(name = "Pitch Tune (DO NOT USE)")
class PitchTune : LinearOpMode() {
    override fun runOpMode() {
        val sampler = Sampler(hardwareMap)

        waitForStart()
        val extendTimer = ElapsedTime()
        while (opModeIsActive()) {
            if (extendTimer.seconds() < 1) {
                sampler.extend()
            } else {
                sampler.setState(Sampler.State.GRAB_SAMPLE)
            }
            sampler.write()
        }
    }
}