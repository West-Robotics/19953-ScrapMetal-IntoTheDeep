package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@TeleOp(name = "ARM RESET")
class ArmReset : LinearOpMode() {
    override fun runOpMode() {
        val sampler = Sampler(hardwareMap)
        waitForStart()
        sampler.state = Sampler.State.DEBUG
        while (opModeIsActive()) {
            sampler.write()
            sampler.updateProfiled()
        }
    }
}
