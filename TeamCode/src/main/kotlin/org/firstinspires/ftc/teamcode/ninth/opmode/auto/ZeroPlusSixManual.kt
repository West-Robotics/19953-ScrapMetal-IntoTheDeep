package org.firstinspires.ftc.teamcode.ninth.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
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
import com.scrapmetal.util.control.pathing.withSpeed
import com.scrapmetal.util.hardware.SMGamepad
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
import org.firstinspires.ftc.teamcode.ninth.opmode.auto.ZeroPlusSixManual.AutoState.*
import kotlin.math.abs

@Autonomous(name="0+6 Manual")
class ZeroPlusSixManual : LinearOpMode() {
    enum class AutoState {
        TO_PRELOAD,
        PRELOAD,
        SPIKE,
        LOWER_TO_SPIKE,
        SPIKE_EXT,
        SCORE,
        DECISION,
        SUB_INTAKE,
        SUB_EXT,
        SUB_GRAB,
        PARK,
        STOP,
    }

    override fun runOpMode() {
        val driver = SMGamepad(gamepad1)
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

        val samps = mutableListOf<Pose2d>()
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
            .transition({ follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.5) }, SUB_INTAKE)
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
                    if (sampCount < 4) {
                        LinePoint(intake[sampCount-1].first.position) lineTo
                            LinePoint(score.position) withHeading Constant(score.heading)
                    } else {
                        SplinePoint(drivetrain.getPoseAndVelo().first.position, -30.0, 0.0) splineTo
                            SplinePoint(score.position, score.heading*Vector2d(20.0, 0.0)) withHeading Linear(Rotation2d(180.0), score.heading)
                    }
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
            .transition({ sampCount >= 4 }, SUB_INTAKE)

            .state(SUB_INTAKE)
            .onEnter {
                sampCount = 4 // TODO: REMOVE
                follower.follow(
                    SplinePoint(score.position, Rotation2d(180.0) *score.heading* Vector2d(30.0, 0.0)) splineTo
                        SplinePoint(Vector2d(72.0, 72.0) + samps[sampCount-4].position + Vector2d(-28.0, -0.5), Vector2d(60.0, 0.0)) withHeading Linear(score.heading, Rotation2d(180.0))
                )
                lift.preset = BOTTOM
            }
            .transitionTimed(0.4)
            .state(SUB_EXT)
            .onEnter {
                sampler.state = EXTING_SAMP_UH
                sampler.setRoll(samps[sampCount-4].heading.theta)
            }
            .transition({ follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.2) })
            .minimumTransitionTimed(1.8)

            .state(SUB_GRAB)
            .onEnter {
                sampler.state = PRIME_SAMP
            }
            .transitionTimed(0.8)
            .waitState(0.2, SCORE)
            .onEnter { sampler.state = GRAB_SAMP }

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
        var x = 0.0
        var y = 0.0
        var ang = 0.0
        while (opModeInInit()) {
            driver.update()
            if (driver.right.rising) {
                x += 0.5
            }
            if (driver.left.rising) {
                x -= 0.5
            }
            if (driver.up.rising) {
                y += 0.5
            }
            if (driver.down.rising) {
                y -= 0.5
            }
            if (driver.lb.rising) {
                ang += 10.0
            }
            if (driver.rb.rising) {
                ang -= 10.0
            }
            if (driver.a.rising) {
                samps.add(Pose2d(x, y, ang))
            }
            for (samp in samps) {
                telemetry.addData("x", samp.position.x)
                telemetry.addData("y", samp.position.y)
                telemetry.addData("ang", samp.heading.theta)
            }
            telemetry.addData("x", x)
            telemetry.addData("y", y)
            telemetry.addData("ang", ang)
            telemetry.update()
        }
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
