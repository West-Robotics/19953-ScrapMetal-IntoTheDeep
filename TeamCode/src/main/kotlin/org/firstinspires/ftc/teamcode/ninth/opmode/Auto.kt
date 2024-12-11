package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "NinthAuto")
class Auto: LinearOpMode() {

    enum class SamplerState {
        SAMPLER_STOW,
        SAMPLER_EXTEND,
        SAMPLER_GRAB,
        SAMPLER_SPIT,
        SAMPLER_HOLD,
        SAMPLER_SCORE,
    }
    var samplerState = SamplerState.SAMPLER_STOW

    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)

        waitForStart()
        val timer = ElapsedTime()

        while (opModeIsActive()) {
            if (timer.seconds() <= 8.0) {
                drivetrain.setVelocity(0.1, 0.0, 0.0)
                lift.runToPos(43.0 - 7.5, lift.getHeight())
                sampler.stow() // stow on purpose, not hold
            }
            if (8.0 < timer.seconds() && timer.seconds() <= 12.0) {
                drivetrain.setVelocity(0.0, 0.0, 0.0)
                lift.runToPos(43.0 - 7.5, lift.getHeight())
                sampler.score()
            }
            if (12.0 < timer.seconds() && timer.seconds() <= 15.0) {
                drivetrain.setVelocity(-0.2, 0.0, 0.0)
                lift.runToPos(43.0 - 7.5, lift.getHeight())
                sampler.stow()
            }
            if (15.0 < timer.seconds()) {
                drivetrain.setVelocity(0.0, 0.0, 0.0)
                lift.runToPos(0.0, lift.getHeight())
                sampler.stow()
            }

            drivetrain.write()
            lift.write()
        }
    }
}
