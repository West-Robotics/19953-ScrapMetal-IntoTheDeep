package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.pControl
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@Autonomous(name = "0+4")
class ZeroPlusFour : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap)
        val lift = Lift(hardwareMap)
        val sampler = Sampler(hardwareMap)

        val translationGain = 1/24.0
        val headingGain = 1/90.0

        waitForStart()
        drivetrain.setPose(0.0, 0.0, 90.0)
        val timer = ElapsedTime()
        var profile1Triggered = false
        var profile2Triggered = false
        var profile3Triggered = false
        var profile4Triggered = false
        var profile5Triggered = false
        var profile6Triggered = false
        var profile7Triggered = false
        var profile8Triggered = false
        var profile9Triggered = false

        while (opModeIsActive()) {
            val pose = drivetrain.getPose()
            // score preload
            if (timer.seconds() <= 4.0) {
                if (!profile1Triggered) {
                    lift.setPreset(Lift.Preset.HIGH)
                    profile1Triggered = true
                }
                lift.updateProfiled(lift.getHeight())
                val fieldFrameEffort = Pose2d(
                    pControl(translationGain, 0.0, pose.position.x),
                    pControl(translationGain, -12.0, pose.position.y),
                    pControl(headingGain, 90.0, pose.heading.theta),
                )
                drivetrain.setEffort(Pose2d(pose.heading.inverse()*fieldFrameEffort.position))
                sampler.stow() // stow on purpose, not hold
            }
            // backup and lower
            if (4.0 < timer.seconds() && timer.seconds() <= 8.0) {
                if (!profile2Triggered) {
                    lift.setPreset(Lift.Preset.BOTTOM)
                    profile2Triggered = true
                }
                lift.updateProfiled(lift.getHeight())
                val fieldFrameEffort = Pose2d(
                    pControl(translationGain, 8.0, pose.position.x),
                    pControl(translationGain, 0.0, pose.position.y),
                    pControl(headingGain, 90.0, pose.heading.theta),
                )
                drivetrain.setEffort(Pose2d(pose.heading.inverse()*fieldFrameEffort.position))
                sampler.stow() // stow on purpose, not hold
            }

            drivetrain.write()
            lift.write()
        }
    }
}