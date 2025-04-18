package org.firstinspires.ftc.teamcode.ninth.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pathing.Constant
import com.scrapmetal.util.control.pathing.Follower
import com.scrapmetal.util.control.pathing.LinePoint
import com.scrapmetal.util.control.pathing.Linear
import com.scrapmetal.util.control.pathing.SplinePoint
import com.scrapmetal.util.control.pathing.lineTo
import com.scrapmetal.util.control.pathing.splineTo
import com.scrapmetal.util.control.pathing.withHeading
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.LENGTH
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.WIDTH
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift.Preset.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.State.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Roll.*
import org.firstinspires.ftc.teamcode.ninth.opmode.auto.ZeroPlusSeven.AutoState.*
import kotlin.math.abs

@Autonomous(name="0+7")
class ZeroPlusSeven : LinearOpMode() {
    enum class AutoState {
        PRELOAD,
        SPIKE,
        SPIKE_EXT,
        DETECT,
        INTAKE,
        SCORE,
        DECISION,
    }

    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage), auto=true)
        val sampler = Sampler(hardwareMap)
        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0)
        limelight.setPollRateHz(80)
        var latestResult = limelight.latestResult
        var sampAngle = 0.0

        val follower = Follower(kN=0.6, kP=0.6, kD=0.05, kTheta=0.09, kOmega=0.005, endDistance=12.0)
        val start = Pose2d(48.0 - LENGTH/2, WIDTH/2, 180.0)
        val score = Pose2d(19.0, 19.0, 180.0 + 45.0)
        val intake = listOf(
            Pair(Pose2d(23.5, 29.0, 180 + 90.0), R90),
            Pair(Pose2d(13.5, 29.0, 180 + 90.0), R90),
            Pair(Pose2d(16.0, 34.5, 180 + 90.0 + 45.0), R45)
        )
        val sub = Pose2d(38.0, 60.0, 180.0)

        var sampCount = 2
        val fsm = StateMachineBuilder()
            .state(PRELOAD)
            .onEnter {
                follower.follow(
                    LinePoint(start.position) lineTo
                            LinePoint(score.position) withHeading Constant(score.heading)
                )
                lift.preset = SAMP_HIGH
                sampler.state = HOLD_SAMP
            }
            .transition { abs(lift.height - SAMP_HIGH.height) < 10.0 }
            // .transitionTimed(1.2)
            .waitState(0.4)
            .onEnter { sampler.state = MOVE_SCORE_SAMP }
            .waitState(0.2)
            .onEnter { sampler.state = PREP_SCORE_SAMP }
            .waitState(0.1)
            .onEnter { sampler.state = SCORE_SAMP; sampCount++ }
            .waitState(0.2)
            .onEnter { sampler.state = STOW }

            .state(SPIKE)
            .onEnter {
                follower.follow(
                    LinePoint(score.position) lineTo
                            LinePoint(intake[sampCount-1].first.position) withHeading Constant(intake[sampCount-1].first.heading)
                )
                lift.preset = BOTTOM
                sampler.setRoll(intake[sampCount-1].second)
            }
            .transition({ lift.height < 12.0 }, { sampler.state = EXTING_SAMP_S })
            .state(SPIKE_EXT)
            .minimumTransitionTimed(1.5)
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) && lift.height < 0.3} // OR TIME LIMIT
            .waitState(0.15)
            .onEnter { sampler.state = PRIME_SAMP_S }
            .waitState(0.1, SCORE)
            .onEnter { sampler.state = GRAB_SAMP_S }

            .state(DETECT)
            .onEnter {
                follower.follow(
                    SplinePoint(score.position, Rotation2d(180.0)*score.heading*Vector2d(20.0, 0.0)) splineTo
                        SplinePoint(sub.position, Vector2d(30.0, 0.0)) withHeading Linear(score.heading, Rotation2d(180.0))
                )
                lift.preset = BOTTOM
            }
            .loop { latestResult = limelight.latestResult }
            .minimumTransitionTimed(2.0)
            .transition({ follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.2) && latestResult != null }) {
                val pose = drivetrain.getPoseAndVelo().first
                // TODO: this rotation compensation is not technically correct
                val sampPos = pose.position - Vector2d(22.0, 0.0) + Rotation2d(180.0)*pose.heading*Vector2d(latestResult.pythonOutput[0], latestResult.pythonOutput[1])
                follower.follow(
                    LinePoint(pose.position) lineTo
                        LinePoint(sampPos) withHeading Constant(180.0)
                )
                sampAngle = latestResult.pythonOutput[2]
            }

            .state(INTAKE)
            .onEnter {
                sampler.state = EXTING_SAMP
                sampler.setRoll(sampAngle)
            }
            .minimumTransitionTimed(1.0)
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) }
            .waitState(0.40)
            .onEnter { sampler.state = PRIME_SAMP }
            .waitState(0.24, SCORE)
            .onEnter { sampler.state = GRAB_SAMP }

            .state(SCORE)
            .onEnter {
                follower.follow(
                    if (sampCount < 4) {
                        LinePoint(intake[sampCount-1].first.position) lineTo
                            LinePoint(score.position) withHeading Constant(score.heading)
                    } else {
                        SplinePoint(drivetrain.getPoseAndVelo().first.position, -30.0, 0.0) splineTo
                            SplinePoint(score.position, score.heading*Vector2d(20.0, 0.0)) withHeading Linear(Rotation2d(180.0), score.heading)
                    }
                )
                lift.preset = SAMP_HIGH
                sampler.state = HOLD_SAMP
            }
            .transition { abs(lift.height - SAMP_HIGH.height) < 10.0 }
            // .transitionTimed(1.2)
            .waitState(0.4)
            .onEnter { sampler.state = MOVE_SCORE_SAMP }
            .waitState(0.2)
            .onEnter { sampler.state = PREP_SCORE_SAMP }
            .waitState(0.1)
            .onEnter { sampler.state = SCORE_SAMP; sampCount++ }
            .waitState(0.2)
            .onEnter { sampler.state = STOW }
            // YOU CAN'T DO AN IF STATEMENT HERE BECAUSE IT ONLY RUNS ONCE IN THE BUILDER
            .waitState(0.1)

            .state(DECISION)
            .transition({ sampCount < 4 }, SPIKE)
            .transition({ sampCount >= 4 }, DETECT)

            .build()

        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
//        lift.updateProfiled(lift.getHeight(), telemetry)
//        telemetry.update()
        waitForStart()
        drivetrain.setPose(start)
        limelight.start()
        fsm.start()
        while (opModeIsActive()) {
            drivetrain.read()
            lift.read()

            fsm.update()
            val (pose, velo) = drivetrain.getPoseAndVelo()
            drivetrain.setEffort(follower.update(pose, velo))
            lift.updateProfiled(lift.height)
            sampler.updateProfiled()

            telemetry.addData("heading", pose.heading.theta)

            drivetrain.write()
            lift.write()
            sampler.write()
            telemetry.update()
        }
    }
}
