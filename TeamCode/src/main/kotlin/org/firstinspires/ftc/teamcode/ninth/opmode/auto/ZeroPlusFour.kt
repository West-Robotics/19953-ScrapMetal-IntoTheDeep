package org.firstinspires.ftc.teamcode.ninth.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.pathing.Constant
import com.scrapmetal.util.control.pathing.Follower
import com.scrapmetal.util.control.pathing.LinePoint
import com.scrapmetal.util.control.pathing.lineTo
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
import org.firstinspires.ftc.teamcode.ninth.opmode.auto.ZeroPlusFour.AutoState.*
import kotlin.math.abs

@Autonomous(name="0+4")
class ZeroPlusFour : LinearOpMode() {
    enum class AutoState {
        TO_PRELOAD,
        PRELOAD,
        SPIKE,
        LOWER_TO_SPIKE,
        SPIKE_EXT,
        SCORE,
        DECISION,
        PARK,
        STOP,
    }

    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage), auto=true)
        val sampler = Sampler(hardwareMap)

        val follower = Follower(kN=0.5, kP=0.5, kD=0.05, kTheta=0.08, kOmega=0.005, endDistance=12.0)
        val start = Pose2d(48.0 - LENGTH/2, WIDTH/2, 180.0)
        val score = Pose2d(16.5, 16.0, 180.0 + 45.0)
        val intake = listOf(
            Pair(Pose2d(19.0, 17.5, 180 + 80.0), R90),
            Pair(Pose2d(16.0, 17.2, 180 + 95.0), R90),
            Pair(Pose2d(17.9, 21.2, 180 + 90.0 + 30.0), R45)
        )

        var sampCount = 0
        var release = false
        val fsm = StateMachineBuilder()
            .state(TO_PRELOAD)
            .onEnter {
                follower.follow(
                    LinePoint(start.position) lineTo
                            LinePoint(score.position) withHeading Constant(score.heading) withSpeed 0.7
                )
                sampler.state = HOLD_SAMP
            }
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.5) }
            .state(PRELOAD)
            .onEnter {
                lift.preset = HIGH_SAMP_HIGH
            }
            .transition { abs(lift.height - SAMP_HIGH.height) < 10.0 }
            .waitState(0.6)
            .onEnter { sampler.state = MOVE_SCORE_SAMP }
            .waitState(0.1)
            .onEnter { sampler.state = AUTO_SCORE_SAMP; sampCount++ }
            .waitState(0.2)
            .onEnter { sampler.state = STOW }
            // .waitState(1.2)
            // .onEnter { sampler.state = PREP_SCORE_SAMP }
            // .waitState(0.1)
            // .onEnter { sampler.state = SCORE_SAMP; sampCount++ }
            // .waitState(0.2)
            // .onEnter { sampler.state = STOW }

            .state(SPIKE)
            .onEnter {
                follower.follow(
                    LinePoint(score.position) lineTo
                        LinePoint(intake[sampCount-1].first.position) withHeading Constant(intake[sampCount-1].first.heading)
                )
                sampler.setRoll(intake[sampCount-1].second)
            }
            .transitionTimed(0.1)
            .state(LOWER_TO_SPIKE)
            .onEnter { lift.preset = BOTTOM }
            .transition({ lift.height < 12.0 }, { sampler.state = EXTING_SAMP_UH })
            .state(SPIKE_EXT)
            .minimumTransitionTimed(1.0)
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.3) && lift.height < 0.3} // OR TIME LIMIT
            .waitState(0.8)
            .onEnter { sampler.state = PRIME_SAMP }
            .waitState(0.3)
            .onEnter { sampler.state = GRAB_SAMP }

            .state(SCORE)
            .onEnter {
                follower.follow(
                    LinePoint(if (sampCount < 4) intake[sampCount-1].first.position else drivetrain.getPoseAndVelo().first.position) lineTo
                        LinePoint(score.position) withHeading Constant(score.heading) withSpeed 0.7
                )
                lift.preset = HIGH_SAMP_HIGH
                sampler.state = HOLD_SAMP
            }
            .transition { abs(lift.height - SAMP_HIGH.height) < 2.0 }
            .waitState(0.6)
            .onEnter { sampler.state = MOVE_SCORE_SAMP }
            .waitState(0.1)
            .onEnter { sampler.state = AUTO_SCORE_SAMP; sampCount++ }
            .waitState(0.2)
            .onEnter { sampler.state = STOW }
            // YOU CAN'T DO AN IF STATEMENT HERE BECAUSE IT ONLY RUNS ONCE IN THE BUILDER
            .waitState(0.1)
            .state(DECISION)
            .transition({ sampCount < 4 }, SPIKE)
            .transition({ sampCount >= 4 }, AutoState.PARK)

            .state(PARK)
            .onEnter{
                follower.follow(
                    LinePoint(score.position) lineTo LinePoint(43.0, 62.0) withHeading Constant(180.0)
                )
                lift.preset = SAMP_LOW
            }
            .transitionTimed(2.0)
            .state(STOP)
            .onEnter { sampler.state = EXTING_SAMP }
            .afterTime(0.5, { release = true })

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
            if (!release) {
                lift.updateProfiled(lift.height)
            } else {
                lift.effort = 0.0
            }
            sampler.updateProfiled()

            drivetrain.write()
            lift.write()
            sampler.write()
            telemetry.update()
        }
    }
}