package org.firstinspires.ftc.teamcode.ninth.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.util.control.Pose2d
import com.scrapmetal.util.control.Rotation2d
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pathing.drawRobot
import com.sfdev.assembly.state.StateMachineBuilder
import kotlinx.coroutines.runBlocking
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
        DECISION,
        PARK,
    }

    override fun runOpMode() = runBlocking {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage), true)
        val sampler = Sampler(hardwareMap)

        val startPose = Pose2d(48.0 - LENGTH/2, WIDTH/2, 180.0)
        val scorePose = Pose2d(17.0, 17.0, 180.0 + 45.0)
        val intakePoses = listOf(
            Pose2d(13.0, 18.0, 180 + 75.0),
            Pose2d(38.5, 26.5, -40.0),
            Pose2d(28.0, 26.5, -40.0),
        )
        val intakeOffsets = listOf(
            Pose2d(0.0, 0.0, -15.0),
            Pose2d(Rotation2d(130.0)*Vector2d(3.5, 0.0), Rotation2d(0.0)),
            Pose2d(Rotation2d(130.0)*Vector2d(2.5, 0.0), Rotation2d(0.0)),
        )
        var currentTargetPose = startPose
        var transMultiplier = 0.7
        var rotationMultiplier = 0.4
        var sampCount = 0

        val fsm = StateMachineBuilder()
            .state(State.SCORE)
            .onEnter {
                currentTargetPose = scorePose
                sampler.hold()
            }
            .transitionTimed(0.75)
            .waitState(1.1)
            .onEnter { lift.setPreset(Lift.Preset.SAMP_HIGH) }
            .waitState(0.4)
            .onEnter { sampler.score_sample(); sampCount++ }
            .onExit { sampler.stow() }
            // YOU CAN'T DO AN IF STATEMENT HERE BECAUSE IT ONLY RUNS ONCE IN THE BUILDER
            .waitState(0.1)
            .state(State.DECISION)
            .transition({ sampCount < 4 }, State.INTAKE)
            .transition({ sampCount >= 4 }, State.PARK)

            .state(State.INTAKE)
            .onEnter {
                lift.setPreset(Lift.Preset.BOTTOM)
            }
            .transitionTimed(0.5)
            .waitState(0.5)
            .onEnter {
                currentTargetPose = intakePoses[sampCount - 1]
            }
            .waitState(0.5)
            .onEnter { sampler.extend() }
            .waitState(0.75)
            .onEnter {
                with (sampler) {
                    when (sampCount) {
//                        1 -> grab_sample_right_side()
                        1 -> grab_sample_right_side()
                        2 -> grab_sample()
                        3 -> grab_sample()
                    }
                }
            }
            .waitState(1.0, State.SCORE)
            .onEnter {
                currentTargetPose += intakeOffsets[sampCount - 1]
                rotationMultiplier = if (sampCount != 1) 0.25 else 0.18
            }
            .onExit { rotationMultiplier = 1.0 }

            .state(State.PARK)
            .onEnter {
                lift.setPreset(Lift.Preset.BOTTOM)
                sampler.stow()
            }
            .afterTime(0.5) { currentTargetPose = startPose }
            .build()

        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        lift.updateProfiled(lift.getHeight(), telemetry)
        telemetry.update()
        waitForStart()
        drivetrain.setPose(startPose)
        drivetrain.beginPinpoint(this)
        fsm.start()
//        var loopCounter = 0
//        val dashboard = FtcDashboard.getInstance()
        while (opModeIsActive()) {
            lift.read()

            fsm.update()
            drivetrain.setEffort(drivetrain.getPDEffort(
                currentTargetPose,
                maxTransEffort = transMultiplier,
                maxRotEffort = rotationMultiplier,
            ))
            lift.updateProfiled(lift.getHeight(), telemetry)

            drivetrain.write()
            lift.write()
            sampler.write()
//            loopCounter++
//            dashboard.telemetry.addData("loop count", loopCounter)
//            val packet = TelemetryPacket()
//            packet.fieldOverlay()
//                .drawRobot(currentTargetPose)
//            dashboard.sendTelemetryPacket(packet)
            telemetry.update()
        }
    }
}