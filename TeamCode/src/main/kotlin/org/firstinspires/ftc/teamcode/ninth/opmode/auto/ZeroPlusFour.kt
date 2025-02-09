package org.firstinspires.ftc.teamcode.ninth.opmode.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.LENGTH
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.WIDTH
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler

@Autonomous(name="0+4")
class ZeroPlusFour : LinearOpMode() {
    enum class State {
        INTAKE,
        SCORE,
        PARK,
    }

    override fun runOpMode() {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage).coerceAtLeast(1.0))
        val sampler = Sampler(hardwareMap)

        val startPose = Pose2d(48.0 - LENGTH/2, WIDTH/2, 180.0)
        val scorePose = Pose2d(18.0, 18.0, 180.0 + 45.0)
        val intakePoses = listOf(Pose2d(12.0, 18.0, 80.0), Pose2d(10.0, 18.0, 80.0), Pose2d(24.0, 18.0, 110.0))
        intakePoses[1]
        var currentTargetPose = startPose
        var sampCount = 0

        val fsm = StateMachineBuilder()
            .state(State.SCORE)
            .onEnter {
                currentTargetPose = scorePose
                lift.setPreset(Lift.Preset.SAMP_HIGH)
                sampler.hold()
            }
            .afterTime(if (sampCount != 0) 1.5 else 2.0) { sampler.score_sample(); sampCount++ }
            .afterTime(if (sampCount != 0) 1.9 else 2.4) { sampler.stow() }
            .transitionTimed(if (sampCount != 0) 2.0 else 2.5, if (sampCount < 4) State.INTAKE else State.PARK)

            .state(State.INTAKE)
            .onEnter {
                currentTargetPose = intakePoses[sampCount]
                lift.setPreset(Lift.Preset.BOTTOM)
                sampler.extend()
            }
            .transitionTimed(3.0, State.SCORE)

            .state(State.PARK)
            .onEnter {
                currentTargetPose = startPose
                lift.setPreset(Lift.Preset.BOTTOM)
                sampler.stow()
            }
            .build()

        waitForStart()
        // TODO: begin pinpoint
        drivetrain.setPose(0.0, 0.0, 90.0)
        while (opModeIsActive()) {
            lift.read()

            fsm.update()
            drivetrain.setEffort(drivetrain.getPDEffort(currentTargetPose))

            drivetrain.write()
//            lift.write()
//            sampler.write()
        }
    }
}