package org.firstinspires.ftc.teamcode.ninth.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.util.control.Pose2d
import com.sfdev.assembly.state.StateMachineBuilder
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.ninth.LENGTH
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.WIDTH
import org.firstinspires.ftc.teamcode.ninth.opmode.tele.SpecTele.SamplerState
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@Autonomous(name="4+0")
class FourPlusZero : LinearOpMode() {
    enum class State {
        SCORE,
        MORE_DECISION,
        SPIKE,
        SWEEP,
        SWEEP_DECISION,
        INTAKE,
        PARK,
    }

    override fun runOpMode() = runBlocking {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage), true)
        val sampler = Sampler(hardwareMap)

        val startPose = Pose2d(72 + WIDTH/2, LENGTH/2 + 1.0, -90.0)
        val scorePose = Pose2d(74.0, 35.0, -108.0)
        val scoreOffset = Pose2d(-3.0, 0.0, 0.0)
        val neutralPose = Pose2d(72.0, 20.0, 180.0)
        val middlePose2 = Pose2d(72.0 + 0.0, 14.0, 180.0 + 35.0)
        val spikePose = Pose2d(72 + 26.0, 24.0 + 4.0, 180 + 35.0)
        val sweepPose = Pose2d(72 + 26.0, 24.0 + 4.0, 180 - 40.0)
        val spikeOffset = Pose2d(10.0, 0.0, 0.0)
        val intakeAlignPose = Pose2d(72 + 8.0, WIDTH/2 + 1.5 + 0.1, 180.0)
        val intakePose = Pose2d(72 + 12.0, WIDTH/2 + 1.5 + 0.1, 180.0)
        val parkPose = Pose2d(72 + 36.0, WIDTH/2 + 1.5 + 0.1, 180.0)
        var currentTargetPose = startPose
        var transMultiplier = 0.35
        var rotationMultiplier = 1.0
        var specCount = 0
        var spikeCount = 0

        val fsm = StateMachineBuilder()
            .state(State.SCORE)
            .onEnter {
                currentTargetPose = scorePose + scoreOffset * specCount.toDouble()
                lift.setPreset(Lift.Preset.SPEC_HIGH)
                sampler.hold_specimen()
            }
            .loop { sampler.updateProfiled() }
            .transitionTimed(1.8)
            .waitState(0.5)
            .onEnter { sampler.dip_specimen() }
            .loop { sampler.updateProfiled() }
            .waitState(0.5)
            .onEnter { sampler.retract_specimen() }
            .loop { sampler.updateProfiled(retracting = true) }
            .waitState(0.8)
            .onEnter { sampler.score_specimen(); lift.setPreset(Lift.Preset.SPEC_HIGH_SCORE) }
            .waitState(0.2)
            .onEnter { sampler.release_specimen() }
            .waitState(0.5)
            .onEnter { sampler.stow(); lift.setPreset(Lift.Preset.SPEC_HIGH); specCount++ }
            .waitState(0.5, State.MORE_DECISION)
            .onEnter { lift.setPreset(Lift.Preset.BOTTOM); currentTargetPose = neutralPose }
            .state(State.MORE_DECISION)
            .transition({ specCount == 1 }, State.SPIKE)
            .transition({ specCount in 2..2 }, State.INTAKE)
            .transition({ specCount == 3 }, State.PARK)

            .state(State.SPIKE)
            .onEnter {
                currentTargetPose = spikePose + spikeOffset * spikeCount.toDouble()
                sampler.extend()
            }
            .transitionTimed(0.5)
            .waitState(1.0)
            .onEnter { sampler.grab_sample() }
            .state(State.SWEEP)
            .onEnter {
                currentTargetPose = sweepPose + spikeOffset * spikeCount.toDouble()
                spikeCount++
            }
            .transitionTimed(0.8)
            .waitState(0.3)
            .onEnter { sampler.spit() }
            .state(State.SWEEP_DECISION)
            .transition({ spikeCount == 1 }, State.INTAKE)
            .transition({ spikeCount < 1 }, State.SPIKE)

            .state(State.INTAKE)
            .onEnter {
                currentTargetPose = intakeAlignPose
                transMultiplier = 1.0
                sampler.extend()
            }
            .transitionTimed(1.0)
            .waitState(1.0, State.SCORE)
            .onEnter {
                currentTargetPose = intakePose
                transMultiplier = 0.4
                sampler.grab_sample()
            }
            .onExit { transMultiplier = 0.3 }

            .state(State.PARK)
            .onEnter {
                currentTargetPose = parkPose
                sampler.stow()
            }
            .build()

        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.update()
        waitForStart()
        drivetrain.setPose(startPose)
        drivetrain.beginPinpoint(this)
        fsm.start()
        while (opModeIsActive()) {
            lift.read()

            fsm.update()
            drivetrain.setEffort(drivetrain.getPDEffort(
                currentTargetPose,
                maxTransEffort = transMultiplier,
                maxRotEffort = rotationMultiplier,
            ))
            if (lift.getPreset() == Lift.Preset.SPEC_HIGH_SCORE) {
                lift.updatePid(lift.getHeight())
            } else {
                lift.updateProfiled(lift.getHeight())
            }

            drivetrain.write()
            lift.write()
//            sampler.hold_specimen()
            sampler.write()
            val pose = drivetrain.getPoseAndVelo().first
            telemetry.addData("x", pose.position.x)
            telemetry.addData("y", pose.position.y)
            telemetry.addData("heading", pose.heading.theta)
            telemetry.update()
        }
    }
}
