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

@Autonomous(name="0+6")
class ZeroPlusSix : LinearOpMode() {
    enum class State {
        INTAKE,
        SCORE,
        DECISION,
        SUB_ALIGN,
        SUB_INTAKE,
        PARK,
    }

    override fun runOpMode() = runBlocking {
        val drivetrain = Drivetrain(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage))
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage), true)
        val sampler = Sampler(hardwareMap)

        val startPose = Pose2d(48.0 - LENGTH/2, WIDTH/2, 180.0)
        val scorePose = Pose2d(17.0, 17.0, 180.0 + 45.0)
        val intakePoses = listOf(
            Pose2d(13.0, 19.0, 180 + 75.0),
            Pose2d(21.0, 17.5, 180 + 95.0),
            Pose2d(24.0, 22.5, 180 + 130.0),
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


        val dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        lift.updateProfiled(lift.getHeight(), telemetry)
        telemetry.update()
        var xMarks1 = 0
        var yMarks1 = 0
        var xMarks2 = 0
        var yMarks2 = 0
        while (!isStarted) {
            if (gamepad1.a) {
                if (gamepad1.dpad_up) yMarks1 += 1
                if (gamepad1.dpad_down) yMarks1 -= 1
                if (gamepad1.dpad_left) xMarks1 -= 1
                if (gamepad1.dpad_right) xMarks1 += 1
            }
            if (gamepad1.x) {
                if (gamepad1.dpad_up) yMarks2 += 1
                if (gamepad1.dpad_down) yMarks2 -= 1
                if (gamepad1.dpad_left) xMarks2 -= 1
                if (gamepad1.dpad_right) xMarks2 += 1
            }
            telemetry.addData("x marks 1", xMarks1)
            telemetry.addData("y marks 1", yMarks1)
            telemetry.addData("x marks 2", xMarks2)
            telemetry.addData("y marks 2", yMarks2)
            telemetry.update()
        }
        val sub1Pos = Vector2d(
            (0..xMarks1).fold(0.0) { sum, mark ->
                sum + when {
                    mark == 0 -> 0.0
                    mark % 2 == 0 -> 1.0
                    else -> 1.5 - 1.0/8.0
                }
            },
            (0..yMarks1).fold(0.0) { sum, mark ->
                sum + when {
                    mark == 0 -> 0.0
                    mark % 2 == 0 -> 1.0
                    else -> 1.5 - 1.0/8.0
                }
            },
        )
        val sub2Pos = Vector2d(
            (0..xMarks2).fold(0.0) { sum, mark ->
                sum + when {
                    mark == 0 -> 0.0
                    mark % 2 == 0 -> 1.0
                    else -> 1.5 - 1.0/8.0
                }
            },
            (0..yMarks2).fold(0.0) { sum, mark ->
                sum + when {
                    mark == 0 -> 0.0
                    mark % 2 == 0 -> 1.0
                    else -> 1.5 - 1.0/8.0
                }
            },
        )

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
