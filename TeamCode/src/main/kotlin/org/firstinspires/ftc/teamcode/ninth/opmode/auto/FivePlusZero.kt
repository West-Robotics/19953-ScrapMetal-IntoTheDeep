package org.firstinspires.ftc.teamcode.ninth.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pathing.Constant
import com.scrapmetal.util.control.pathing.Follower
import com.scrapmetal.util.control.pathing.LinePoint
import com.scrapmetal.util.control.pathing.SplinePoint
import com.scrapmetal.util.control.pathing.lineTo
import com.scrapmetal.util.control.pathing.splineTo
import com.scrapmetal.util.control.pathing.withHeading
import com.scrapmetal.util.control.pathing.withSpeed
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
import org.firstinspires.ftc.teamcode.ninth.opmode.auto.FivePlusZero.AutoState.*
import kotlin.math.abs

@Autonomous(name="5+0")
class FivePlusZero : LinearOpMode() {
    enum class AutoState {
        PREP_SCORE,
        SCORE,
        DECISION,
        LOWER,
        PRE_INTAKE,
        INTAKE,
        GRAB,
        SWEEP_INTAKE,
        BACKUP_DECISION,
        BACKUP,
        SWEEP_OUTPUT,
        SWEEP_DECISION,
    }

    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage), auto=true)
        val sampler = Sampler(hardwareMap)

        val follower = Follower(kN=0.5, kP=0.5, kD=0.05, kTheta=0.08, kOmega=0.005, endDistance=12.0)
        val start = Pose2d(0.0, 0.0, 90.0)
        val score = Pose2d(-2.0, 26.2, 90.0)
        val offset = Pose2d(-1.0, 0.0, 0.0)
        val intake = Pose2d(34.0, 0.5, 90.0)
        val sweep = Pose2d(19.0, 19.0, 180 + 40.0)
        val sweepOffset = Pose2d(10.5, 0.0, 0.0)
        val hp = Pose2d(19.0, 16.0, 180 - 40.0)

        val GRAB_WAIT = 0.14

        var specCount = 0
        var sweepCount = 0
        val fsm = StateMachineBuilder()
            .state(PREP_SCORE)
            .onEnter {
                follower.follow(
                    LinePoint(drivetrain.getPoseAndVelo().first.position) lineTo
                        LinePoint(score.position + offset.position*(specCount.toDouble())) withSpeed (if (specCount == 0) 0.6 else 1.0) withHeading Constant(90.0)
                )
                lift.preset = SPEC_HIGH
                sampler.state = PREP_SCORE_SPEC
            }
            .minimumTransitionTimed(1.0)
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) }

            .state(SCORE)
            .onEnter {
                lift.preset = SPEC_HIGH_SCORE
                specCount++
            }
            .transitionTimed(0.4)

            .state(DECISION)
            .transition({ specCount == 1 }, LOWER)
            .transition({ specCount > 1 }, INTAKE)

            .state(LOWER)
            .onEnter {
                follower.follow(
                    LinePoint(drivetrain.getPoseAndVelo().first.position) lineTo
                            LinePoint(sweep.position + sweepOffset.position*(sweepCount.toDouble())) withHeading Constant(sweep.heading)
                )
                lift.preset = BOTTOM
            }
            .transitionTimed(0.5)

            .state(SWEEP_INTAKE)
            .onEnter {
                follower.follow(
                    LinePoint(drivetrain.getPoseAndVelo().first.position) lineTo
                        LinePoint(sweep.position + sweepOffset.position*(sweepCount.toDouble())) withHeading Constant(sweep.heading)
                )
                sampler.state = EXTING_SAMP_UH
                sampler.setRoll(RCW45)
            }
            .minimumTransitionTimed(1.2)
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) }
            .waitState(0.3)
            .onEnter { sampler.state = PRIME_SAMP }
            .waitState(GRAB_WAIT, SWEEP_OUTPUT)
            .onEnter { sampler.state = GRAB_SAMP }
            .state(BACKUP_DECISION)
            .transition({ follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) && sweepCount < 2 }, SWEEP_OUTPUT)
            .transition({ follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) && sweepCount == 2 }, BACKUP)

            .state(BACKUP)
            .onEnter {
                follower.follow(
                    LinePoint(drivetrain.getPoseAndVelo().first.position) lineTo
                            LinePoint(sweep.position + sweepOffset.position*1.5) withHeading Constant(
                        hp.heading
                    )
                )
            }
            .transitionTimed(0.6)
            .waitState(0.3)
            .onEnter { sampler.state = PRIME_SAMP }
            .waitState(GRAB_WAIT, SWEEP_OUTPUT)
            .onEnter { sampler.state = GRAB_SAMP }


            .state(SWEEP_OUTPUT)
            .onEnter {
                follower.follow(
                    LinePoint(drivetrain.getPoseAndVelo().first.position) lineTo
                        LinePoint(hp.position + if (sweepCount == 2) Vector2d(-8.0, 0.0) else Vector2d(0.0, 0.0) + sweepOffset.position*(sweepCount.toDouble())) withHeading Constant(hp.heading)
                )
                sweepCount++
            }
            .transition({ follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) }, { sampler.state = EXTING_SAMP_UH })
            .state(SWEEP_DECISION)
            .minimumTransitionTimed(0.1)
            .transition({ sweepCount < 2 }, SWEEP_INTAKE)
            .transition({ sweepCount == 2 }, PRE_INTAKE)

            // .transition({ specCount == 1 }, SWEEP_INTAKE)
            // .transition({ specCount > 1 }, )

            .state(PRE_INTAKE)
            .onEnter {
                follower.follow(
                    LinePoint(drivetrain.getPoseAndVelo().first.position) lineTo
                        LinePoint(intake.position + Vector2d(0.0, 12.0)) withHeading Constant(hp.heading)
                )
            }
            .transitionTimed(1.0)

            .state(INTAKE)
            .onEnter {
                follower.follow(
                    SplinePoint(score.position, Vector2d(30.0, -30.0)) splineTo
                        SplinePoint(intake.position, Vector2d(0.0, -60.0)) withHeading Constant(90.0) withSpeed 0.7
                )
                lift.preset = BOTTOM
                sampler.state = PRIME_SPEC
            }
            .minimumTransitionTimed(1.8)
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) }
            .state(GRAB)
            .onEnter { sampler.state = GRAB_SPEC }
            .transitionTimed(GRAB_WAIT, PREP_SCORE)

            .build()

//        val dashboard = FtcDashboard.getInstance()
//        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
//        lift.updateProfiled(lift.getHeight(), telemetry)
//        telemetry.update()
        waitForStart()
        drivetrain.setPose(start)
        fsm.start()
        while (opModeIsActive()) {
            drivetrain.read()
            lift.read()

            fsm.update()
            val (pose, velo) = drivetrain.getPoseAndVelo()
            drivetrain.setEffort(follower.update(pose, velo))
            lift.updateProfiled(lift.height)
            sampler.updateProfiled()

            drivetrain.write()
            lift.write()
            sampler.write()
            telemetry.update()
        }
    }
}
