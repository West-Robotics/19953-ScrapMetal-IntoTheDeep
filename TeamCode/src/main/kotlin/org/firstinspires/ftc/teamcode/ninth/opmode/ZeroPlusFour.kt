package org.firstinspires.ftc.teamcode.ninth.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
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

        val translationGain = 2/24.0
        val headingGain = 8/90.0
        val scorePose = Pose2d(9.0, 40.0, 90.0 + 45.0)
//        val preScorePose = Pose2d(9.0, 41.0, -45.0)
        val extend1Pose = Pose2d(29.0, 3.0, -107.0)
        val extend2Pose = Pose2d(27.75, 12.0, -107.0)
        val extend3Pose = Pose2d(27.75, 20.0, -107.0)
        val intakeOffset = Pose2d(Rotation2d(180.0)*extend1Pose.heading*Vector2d(6.0, 0.0), Rotation2d())
        val parkPose = Pose2d(53.0, 12.0, 90.0)
        val slowLimit = 1.0

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
            if (timer.seconds() < 28.0) {
                lift.updateProfiled(lift.getHeight())
            } else {
                lift.setEffort(0.0)
            }
            val fieldFrameEffort = when (timer.seconds()) {
                // to preload
                in 0.0..1.7 -> Pose2d(
                    pControl(translationGain, scorePose.position.x, pose.position.x).coerceIn(-slowLimit..slowLimit),
                    pControl(translationGain, scorePose.position.y, pose.position.y).coerceIn(-slowLimit..slowLimit),
                    pControl(headingGain, scorePose.heading.theta, pose.heading.theta, true),
                ).also {
                    if (!profile1Triggered) {
                        lift.setPreset(Lift.Preset.HIGH)
                        profile1Triggered = true
                    }
                    sampler.hold()
                }
                // score
                in 1.7..2.0 -> Pose2d(
                    pControl(translationGain, scorePose.position.x, pose.position.x).coerceIn(-slowLimit..slowLimit),
                    pControl(translationGain, scorePose.position.y, pose.position.y).coerceIn(-slowLimit..slowLimit),
                    pControl(headingGain, scorePose.heading.theta, pose.heading.theta, true),
                ).also { sampler.score_sample() }
                // lower, extend, aim for s1
                in 2.0..4.5 -> Pose2d(
                    pControl(translationGain, extend1Pose.position.x, pose.position.x),
                    pControl(translationGain, extend1Pose.position.y, pose.position.y),
                    pControl(headingGain, extend1Pose.heading.theta, pose.heading.theta, true),
                ).also {
                    if (!profile2Triggered) {
                        lift.setPreset(Lift.Preset.BOTTOM)
                        profile2Triggered = true
                    }
                    sampler.extend()
                }

                // intake s1 (and drive forwards)
                in 4.5..5.5 -> (extend1Pose + intakeOffset).let { Pose2d(
                    pControl(translationGain, it.position.x, pose.position.x).coerceIn(-0.2, 0.2),
                    pControl(translationGain, it.position.y, pose.position.y).coerceIn(-0.2, 0.2),
                    pControl(headingGain, it.heading.theta, pose.heading.theta, true),
                ).also { sampler.grab_sample() }}
                // retract, raise, drive to scoring
                in 5.5..7.7 -> Pose2d(
                    pControl(translationGain, scorePose.position.x, pose.position.x),
                    pControl(translationGain, scorePose.position.y, pose.position.y),
                    pControl(headingGain, scorePose.heading.theta, pose.heading.theta, true),
                ).also {
                    if (!profile3Triggered) {
                        lift.setPreset(Lift.Preset.HIGH)
                        profile3Triggered = true
                    }
                    sampler.hold()
                }
                // score
                in 7.7..8.0 -> Pose2d(
                    pControl(translationGain, scorePose.position.x, pose.position.x),
                    pControl(translationGain, scorePose.position.y, pose.position.y),
                    pControl(headingGain, scorePose.heading.theta, pose.heading.theta, true),
                ).also { sampler.score_sample() }
                // lower, extend, aim for s2
                in 8.0..10.5 -> Pose2d(
                    pControl(translationGain, extend2Pose.position.x, pose.position.x),
                    pControl(translationGain, extend2Pose.position.y, pose.position.y),
                    pControl(headingGain, extend2Pose.heading.theta, pose.heading.theta, true),
                ).also {
                    if (!profile4Triggered) {
                        lift.setPreset(Lift.Preset.BOTTOM)
                        profile4Triggered = true
                    }
                    sampler.extend()
                }



                // intake s2 (and drive forwards)
                in 10.5..11.5 -> (extend2Pose + intakeOffset).let { Pose2d(
                    pControl(translationGain, it.position.x, pose.position.x).coerceIn(-0.2, 0.2),
                    pControl(translationGain, it.position.y, pose.position.y).coerceIn(-0.2, 0.2),
                    pControl(headingGain, it.heading.theta, pose.heading.theta, true),
                ).also { sampler.grab_sample() }}
                // retract, raise, drive to scoring
                in 11.5..13.7 -> Pose2d(
                    pControl(translationGain, scorePose.position.x, pose.position.x),
                    pControl(translationGain, scorePose.position.y, pose.position.y),
                    pControl(headingGain, scorePose.heading.theta, pose.heading.theta, true),
                ).also {
                    if (!profile5Triggered) {
                        lift.setPreset(Lift.Preset.HIGH)
                        profile5Triggered = true
                    }
                    sampler.hold()
                }
                // score
                in 13.7..14.0 -> Pose2d(
                    pControl(translationGain, scorePose.position.x, pose.position.x),
                    pControl(translationGain, scorePose.position.y, pose.position.y),
                    pControl(headingGain, scorePose.heading.theta, pose.heading.theta, true),
                ).also { sampler.score_sample() }
                // lower, extend, aim for s3
                in 14.0..16.5 -> Pose2d(
                    pControl(translationGain, extend3Pose.position.x, pose.position.x),
                    pControl(translationGain, extend3Pose.position.y, pose.position.y),
                    pControl(headingGain, extend3Pose.heading.theta, pose.heading.theta, true),
                ).also {
                    if (!profile6Triggered) {
                        lift.setPreset(Lift.Preset.BOTTOM)
                        profile6Triggered = true
                    }
                    sampler.extend()
                }




                // intake s3 (and drive forwards)
                in 16.5..17.5 -> (extend3Pose + intakeOffset).let { Pose2d(
                    pControl(translationGain, it.position.x, pose.position.x).coerceIn(-0.2, 0.2),
                    pControl(translationGain, it.position.y, pose.position.y).coerceIn(-0.2, 0.2),
                    pControl(headingGain, it.heading.theta, pose.heading.theta, true),
                ).also { sampler.grab_sample() }}
                // retract, raise, drive to scoring
                in 17.5..19.2 -> Pose2d(
                    pControl(translationGain, scorePose.position.x, pose.position.x),
                    pControl(translationGain, scorePose.position.y, pose.position.y),
                    pControl(headingGain, scorePose.heading.theta, pose.heading.theta, true),
                ).also {
                    if (!profile7Triggered) {
                        lift.setPreset(Lift.Preset.HIGH)
                        profile7Triggered = true
                    }
                    sampler.hold()
                }
                // score
                in 19.2..19.5 -> Pose2d(
                    pControl(translationGain, scorePose.position.x, pose.position.x),
                    pControl(translationGain, scorePose.position.y, pose.position.y),
                    pControl(headingGain, scorePose.heading.theta, pose.heading.theta, true),
                ).also { sampler.score_sample() }
                // go to low basket height, do not extend, park
                in 19.5..22.0 -> Pose2d(
                    pControl(translationGain, parkPose.position.x, pose.position.x),
                    pControl(translationGain, parkPose.position.y, pose.position.y),
                    pControl(headingGain, parkPose.heading.theta, pose.heading.theta, true),
                ).also {
                    if (!profile8Triggered) {
                        lift.setPreset(Lift.Preset.LOW)
                        profile8Triggered = true
                    }
                    sampler.stow()
                }
                // extend
                in 22.0..33.0 -> Pose2d(
                    pControl(translationGain, parkPose.position.x, pose.position.x),
                    pControl(translationGain, parkPose.position.y, pose.position.y),
                    pControl(headingGain, parkPose.heading.theta, pose.heading.theta, true),
                ).also {
                    sampler.extend()
                }
                else -> Pose2d().also { sampler.stow() }
            }
            drivetrain.setEffort(Pose2d(pose.heading.inverse()*fieldFrameEffort.position, fieldFrameEffort.heading))

            drivetrain.write()
            lift.write()
            sampler.write()
        }
    }
}