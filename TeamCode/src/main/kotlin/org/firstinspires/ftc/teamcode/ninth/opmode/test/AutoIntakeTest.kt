package org.firstinspires.ftc.teamcode.ninth.opmode.test

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pathing.Constant
import com.scrapmetal.util.control.pathing.Follower
import com.scrapmetal.util.control.pathing.LinePoint
import com.scrapmetal.util.control.pathing.lineTo
import com.scrapmetal.util.control.pathing.withHeading
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.State.*
import org.firstinspires.ftc.teamcode.ninth.opmode.test.AutoIntakeTest.State.*

@Autonomous(name="Auto Intake Test")
class AutoIntakeTest : LinearOpMode() {
    enum class State {
        DETECT,
        INTAKE,
        OUTPUT,
        DECISION,
        STOP,
    }

    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val sampler = Sampler(hardwareMap)
        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0)
        var latestResult = limelight.latestResult

        val follower = Follower(kN=0.5, kP=0.5, kD=0.05, kTheta=0.08, kOmega=0.005, endDistance=12.0)
        val outPose = Pose2d(0.0, 0.0, 180.0)
        var sampAngle = 0.0

        follower.follow(LinePoint(outPose.position) lineTo LinePoint(outPose.position) withHeading Constant(180.0))

        var samps = 0

        val fsm = StateMachineBuilder()
            .state(DETECT)
            // time since last update compensation, timestamp, latency, etc.
            .loop { latestResult = limelight.latestResult }
            .transitionTimed(0.5) {
                val pose = drivetrain.getPoseAndVelo().first
                val sampPos = pose.position - Vector2d(21.5, 0.0) + Rotation2d(180.0) *pose.heading*Vector2d(latestResult.pythonOutput[0], latestResult.pythonOutput[1])
                follower.follow(
                    LinePoint(outPose.position) lineTo
                        LinePoint(sampPos) withHeading Constant(180.0)
                )
                sampAngle = latestResult.pythonOutput[2]
            }

            .state(INTAKE)
            .onEnter {
                sampler.state = EXTING_SAMP
                sampler.setRoll(sampAngle)
            }
            .minimumTransitionTimed(1.5)
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.2) }
            .waitState(0.30)
            .onEnter { sampler.state = PRIME_SAMP }
            .waitState(0.14)
            .onEnter { sampler.state = GRAB_SAMP }

            .state(OUTPUT)
            .onEnter {
                follower.follow(
                    LinePoint(drivetrain.getPoseAndVelo().first.position) lineTo
                        LinePoint(outPose.position) withHeading Constant(180.0)
                )
                sampler.state = MOVE_SCORE_SAMP
            }
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.5) }
            .waitState(0.4)
            .onEnter { sampler.state = PREP_SCORE_SAMP }
            .waitState(0.2)
            .onEnter { sampler.state = SCORE_SAMP; samps++ }

            .state(DECISION)
            .transition({ samps < 4 }, DETECT)
            .transition({ samps == 4 }, STOP)

            .state(STOP)

            .build()

//        val dashboard = FtcDashboard.getInstance()
//        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        limelight.start()
        waitForStart()
        drivetrain.setPose(outPose)
        fsm.start()
        while (opModeIsActive()) {
            drivetrain.read()

            fsm.update()
            val (pose, velo) = drivetrain.getPoseAndVelo()
            drivetrain.setEffort(follower.update(pose, velo))
            sampler.updateProfiled()
            drivetrain.write()
            sampler.write()
            telemetry.addData("state", fsm.state)
            telemetry.addData("x", latestResult.pythonOutput[0])
            telemetry.addData("y", latestResult.pythonOutput[1])
            telemetry.addData("ang", latestResult.pythonOutput[2])
            telemetry.update()
        }
    }
}