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
        val scorePose = Pose2d(18.0, 18.0, 180.0 + 45.0)
        val intakePoses = listOf(
            Pose2d(13.0, 18.0, 180 + 75.0),
            Pose2d(24.0, 17.5, 180 + 95.0),
            Pose2d(24.0, 21.5, 180 + 130.0),
        )
        val intakeOffsets = listOf(
            Pose2d(0.0, 0.0, -15.0),
            Pose2d(0.0, 0.0, 20.0),
            Pose2d(Rotation2d(130.0)*Vector2d(2.5, 0.0), Rotation2d(0.0)),
        )
        var currentTargetPose = startPose
        var transMultiplier = 0.7
        var rotationMultiplier = 0.6
        var sampCount = 0

        val fsm = StateMachineBuilder()
            .state(State.SCORE)
            .onEnter {
                currentTargetPose = scorePose
                lift.setPreset(Lift.Preset.SAMP_HIGH)
                sampler.hold()
            }
            .transitionTimed(1.5)
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
                currentTargetPose = intakePoses[sampCount - 1]
                lift.setPreset(Lift.Preset.BOTTOM)
            }
            .transitionTimed(0.5)
            .waitState(0.5)
            .onEnter { sampler.extend() }
            .waitState(1.0)
            .onEnter {
                with (sampler) {
                    when (sampCount) {
                        1 -> grab_sample_right_side()
                        2 -> grab_sample_left_side()
                        3 -> grab_sample()
                    }
                }
            }
            .waitState(1.0, State.SCORE)
            .onEnter {
                currentTargetPose += intakeOffsets[sampCount - 1]
                rotationMultiplier = 0.2
            }
            .onExit { rotationMultiplier = 1.0 }

            .state(State.PARK)
            .onEnter {
                currentTargetPose = startPose
                lift.setPreset(Lift.Preset.BOTTOM)
                sampler.stow()
            }
            .build()

        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        lift.updateProfiled(lift.getHeight(), telemetry)
        telemetry.update()
        waitForStart()
//        drivetrain.beginPinpoint(this)
        drivetrain.setPose(startPose)
        fsm.start()
//        var loopCounter = 0
//        val dashboard = FtcDashboard.getInstance()
        while (opModeIsActive()) {
            drivetrain.read()
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