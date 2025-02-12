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
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@Autonomous(name="5+0")
class FivePlusZero : LinearOpMode() {
    enum class State {
        SCORE,
        MORE_DECISION,
        SPIKE,
        SWEEP,
        SWEEP_DECISION,
        INTAKE_POSITION,
        INTAKE,
        PARK,
    }

    override fun runOpMode() = runBlocking {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage), true)
        val sampler = Sampler(hardwareMap)

        val startPose = Pose2d(72 + WIDTH/2, LENGTH/2, 90.0)
        val scorePose = Pose2d(72 + 12.0, 38.0, 180 - 45.0)
        val scoreOffset = Pose2d(-3.0, 0.0, 0.0)
        val spikePose = Pose2d(72 + 20.0, 24.0 + 6.0, 180 + 30.0)
        val sweepPose = Pose2d(72 + 20.0, 24.0 + 6.0, 180 - 30.0)
        val spikeOffset = Pose2d(10.0, 0.0, 0.0)
        val intakePose = Pose2d(72 + 20.0, 32.0, 180 - 45.0)
        var currentTargetPose = startPose
        var transMultiplier = 0.5
        var rotationMultiplier = 1.0
        var specCount = 0
        var spikeCount = 0

        val fsm = StateMachineBuilder()
            .state(State.SCORE)
            .onEnter {
                currentTargetPose = scorePose + scoreOffset * specCount.toDouble()
                lift.setPreset(Lift.Preset.SPEC_HIGH)
                sampler.prepare_to_score_specimen()
            }
            .transitionTimed(2.0)
            .waitState(0.4)
            .onEnter { lift.setPreset(Lift.Preset.SPEC_HIGH_SCORE) }
            .waitState(0.3)
            .onEnter { sampler.score_specimen(); specCount++ }
            // YOU CAN'T DO AN IF STATEMENT HERE BECAUSE IT ONLY RUNS ONCE IN THE BUILDER
            .state(State.MORE_DECISION)
            .transition({ specCount == 1 }, State.SPIKE)
            .transition({ specCount in 2..4 }, State.INTAKE)
            .transition({ specCount == 5 }, State.PARK)

            .state(State.SPIKE)
            .onEnter {
                currentTargetPose = spikePose + spikeOffset * spikeCount.toDouble()
                lift.setPreset(Lift.Preset.BOTTOM)
                sampler.grab_sample()
            }
            .transitionTimed(1.5)
            .state(State.SWEEP)
            .onEnter {
                currentTargetPose = sweepPose + spikeOffset * spikeCount.toDouble()
                spikeCount++
            }
            .transitionTimed(0.8)
            .waitState(0.3)
            .onEnter { sampler.spit() }
            .state(State.SWEEP_DECISION)
            .transition({ spikeCount == 3 }, State.INTAKE_POSITION)
            .transition({ spikeCount < 3 }, State.SPIKE)

            .state(State.INTAKE_POSITION)
            .onEnter {
                currentTargetPose = intakePose
                lift.setPreset(Lift.Preset.SPEC_INTAKE)
            }
            .transitionTimed(0.8, State.INTAKE)

            .state(State.INTAKE)
            .onEnter {
                currentTargetPose = intakePose
                lift.setPreset(Lift.Preset.SPEC_INTAKE)
                sampler.extend()
            }
            .transitionTimed(0.5)
            .waitState(1.5)
            .onEnter { sampler.grab_specimen() }
            .waitState(0.25, State.SCORE)
            .onEnter { sampler.lift_specimen() }

            .state(State.PARK)
            .onEnter {
                currentTargetPose = startPose
                lift.setPreset(Lift.Preset.BOTTOM)
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
//            lift.write()
//            sampler.write()
        }
    }
}
