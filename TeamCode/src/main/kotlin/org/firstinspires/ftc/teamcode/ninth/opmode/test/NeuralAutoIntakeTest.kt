package org.firstinspires.ftc.teamcode.ninth.opmode.test

import android.telecom.Call.Details
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult
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

@Autonomous(name="Neural Auto Intake Test")
class NeuralAutoIntakeTest : LinearOpMode() {
    enum class State {
        DETECT,
        INTAKE,
        OUTPUT,
        DECISION,
        STOP,
    }

    val H = listOf(
         6.54887442e-06,  1.20749041e-03, -1.45659209e+00,
         1.96303934e-03, -1.02209728e-03, -2.67235992e-01,
        -1.36431746e-04, -5.29416295e-03,  1.00000000e+00,
    )

    fun applyHomography(H: List<Double>, x: Double, y: Double): Vector2d {
        val tx = H[0] * x + H[1] * y + H[2]
        val ty = H[3] * x + H[4] * y + H[5]
        val tz = H[6] * x * H[7] * y + H[8]

        val px = tx/tz
        val py = ty/tz

        return Vector2d(px, py)
    }

    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val sampler = Sampler(hardwareMap)
        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(3)
        limelight.setPollRateHz(7)
        var latestResult = limelight.latestResult
        var sampOfInterest = Pose2d(0.0, 0.0, 0.0)

        val follower = Follower(kN=0.5, kP=0.5, kD=0.05, kTheta=0.08, kOmega=0.005, endDistance=12.0)
        val outPose = Pose2d(0.0, 0.0, 180.0)
        var sampAngle = 0.0

        follower.follow(LinePoint(outPose.position) lineTo LinePoint(outPose.position) withHeading Constant(180.0))

        var samps = 0

        val fsm = StateMachineBuilder()
            .state(DETECT)
            // time since last update compensation, timestamp, latency, etc.
            .loop {
                latestResult = limelight.latestResult
                lateinit var closest: DetectorResult
                var first = true
                for (detection in latestResult.detectorResults) {
                    if (first) {
                        closest = detection
                        first = false
                    } else {
                        telemetry.addData("tx", detection.targetXPixels)
                        telemetry.addData("tx", detection.targetYPixels)
                        val dist = applyHomography(H, detection.targetXPixels, detection.targetYPixels)
                        val currentDist = applyHomography(H, closest.targetXPixels, closest.targetYPixels)
                        if (dist.norm < currentDist.norm) {
                            closest = detection
                        }
                        sampOfInterest = Pose2d(dist.x, dist.y, 180.0)
                    }
                }
            }
            .minimumTransitionTimed(100.0)
            .transition({ latestResult != null }) {
                lateinit var closest: DetectorResult
                var first = true
                for (detection in latestResult.detectorResults) {
                    if (first) {
                        closest = detection
                        first = false
                    } else {
                        val dist = applyHomography(H, detection.targetXPixels, detection.targetYPixels).norm
                        val currentDist = applyHomography(H, closest.targetXPixels, detection.targetYPixels)
                        if (dist < currentDist.norm) {
                            closest = detection
                        }
                    }
                }
                val pose = drivetrain.getPoseAndVelo().first
                // val sampPos = pose.position - Vector2d(21.5, 0.0) + Rotation2d(180.0)*pose.heading*Vector2d(closest.targetXPixels, closest.targetYPixels)
                // follower.follow(
                    // LinePoint(outPose.position) lineTo
                    //         LinePoint(sampPos) withHeading Constant(180.0)
                // )
                // sampAngle = latestResult.pythonOutput[2]
            }

            .state(INTAKE)
            .onEnter {
                sampler.state = EXTING_SAMP
                sampler.setRoll(sampAngle)
            }
            .minimumTransitionTimed(1.0)
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.2) }
            .waitState(0.40)
            .onEnter { sampler.state = PRIME_SAMP }
            .waitState(0.24)
            .onEnter { sampler.state = GRAB_SAMP }

            .state(OUTPUT)
            .onEnter {
                follower.follow(
                    LinePoint(drivetrain.getPoseAndVelo().first.position) lineTo
                            LinePoint(outPose.position) withHeading Constant(180.0)
                )
                sampler.state = MOVE_SCORE_SAMP
            }
            .transition { follower.atEnd(drivetrain.getPoseAndVelo().first.position, 0.2) }
            .waitState(0.4)
            .onEnter { sampler.state = PREP_SCORE_SAMP }
            .waitState(0.2)
            .onEnter { sampler.state = SCORE_SAMP; samps++ }

            .state(DECISION)
            .transition({ samps < 4 }, DETECT)
            .transition({ samps == 4 }, STOP)

            .state(STOP)

            .build()

        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        limelight.start()
        // telemetry.addData("state", fsm.state)
        // telemetry.addData("x", latestResult.pythonOutput[0])
        // telemetry.addData("y", latestResult.pythonOutput[1])
        // telemetry.addData("ang", latestResult.pythonOutput[2])
        // telemetry.update()
        waitForStart()
        drivetrain.setPose(outPose)
        fsm.start()
        while (opModeIsActive()) {
            drivetrain.read()

            fsm.update()
            val (pose, velo) = drivetrain.getPoseAndVelo()
            drivetrain.setEffort(follower.update(pose, velo))
            sampler.updateProfiled()
            // drivetrain.write()
            // sampler.write()
            telemetry.addData("state", fsm.state)
            telemetry.addData("x", sampOfInterest.position.x)
            telemetry.addData("y", sampOfInterest.position.y)
            telemetry.addData("ang", sampOfInterest.heading.theta)
            telemetry.update()
        }
    }
}
