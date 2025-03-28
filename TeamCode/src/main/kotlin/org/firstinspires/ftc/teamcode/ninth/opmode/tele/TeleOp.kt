package org.firstinspires.ftc.teamcode.ninth.opmode.tele

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.ninth.NOM_VOLT
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.State.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Sampler.Roll.*
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift
import org.firstinspires.ftc.teamcode.ninth.robot.subsystem.Lift.Preset.*
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "TeleOp")

class TeleOp: LinearOpMode() {
    val PRIME_WAIT = 0.15
    val GRAB_WAIT = 0.10
    val SCORE_WAIT = 0.2

    var specHeights = false
    var sampHeights = false

    override fun runOpMode() {
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()

        val drivetrain = Drivetrain(hardwareMap)
        // TODO: readd tele
        val lift = Lift(hardwareMap, (NOM_VOLT / hardwareMap.voltageSensor.iterator().next().voltage).coerceAtLeast(1.0))
        val sampler = Sampler(hardwareMap)

        var manual = false
        // TODO: SET TO LOW FOR POST-AUTO
        lift.setPreset(BOTTOM)
        var ITSCLIMBINTIME = false

        var speedDecrease = 0.0
        var turnDecrease = 0.0

        val samplerFSM = StateMachineBuilder()
            .state(STOW)
            .onEnter {
                speedDecrease = 0.0
                turnDecrease = 0.0
                sampHeights = false
            }
            .transition(
                { (currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8)
                    && lift.getPreset() == Lift.Preset.BOTTOM },
                EXTENDING,
                { sampler.setRoll(R0) }
            )
            .transition(
                { ((currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8)
                        || (currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
                        || (currentGamepad1.right_bumper && !previousGamepad1.right_bumper))
                        && lift.getPreset() == Lift.Preset.BOTTOM },
                EXTENDING,
            )

//          ░██████╗░█████╗░███╗░░░███╗██████╗░██╗░░░░░███████╗
//          ██╔════╝██╔══██╗████╗░████║██╔══██╗██║░░░░░██╔════╝
//          ╚█████╗░███████║██╔████╔██║██████╔╝██║░░░░░█████╗░░
//          ░╚═══██╗██╔══██║██║╚██╔╝██║██╔═══╝░██║░░░░░██╔══╝░░
//          ██████╔╝██║░░██║██║░╚═╝░██║██║░░░░░███████╗███████╗
//          ╚═════╝░╚═╝░░╚═╝╚═╝░░░░░╚═╝╚═╝░░░░░╚══════╝╚══════╝

            .state(EXTENDING)
            .onEnter { speedDecrease = 0.0; turnDecrease = 1.5 }
            .transitionTimed(0.50)
            .state(EXTEND)
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                PRIME_SAMP,
            )
            .transition({ currentGamepad2.x && !previousGamepad2.x }, STOW)

            // speed_decrease = 0.75
            // turn_decrease = 1.5
            .state(PRIME_SAMP)
            .transitionTimed(PRIME_WAIT)
            .state(GRAB_SAMP)
            .transitionTimed(GRAB_WAIT, PICKED)

            .state(PICKED)
            .onEnter{ speedDecrease = 0.0; turnDecrease = 0.0 }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                HOLD_SAMP,
            )
            .transition(
                { currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8 },
                EXTEND,
            )
            .transition({ currentGamepad2.x && !previousGamepad2.x }, STOW)

            .state(HOLD_SAMP)
            .onEnter { sampHeights = true }
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                PREP_SCORE_SAMP,
            )
            .transition(
                {
                    (lift.getPreset() == Lift.Preset.SAMP_HIGH ||
                            lift.getPreset() == Lift.Preset.SAMP_LOW) &&
                            abs(lift.getHeight() - lift.getPreset().height) < 0.75
                },
                PREP_SCORE_SAMP,
            )
            .transition({ currentGamepad2.x && !previousGamepad2.x }, STOW)

            .state(PREP_SCORE_SAMP)
            .transition(
                { currentGamepad1.left_trigger > 0.8 && previousGamepad1.left_trigger <= 0.8 },
                SCORE_SAMP,
            )
            .transition({ currentGamepad2.x && !previousGamepad2.x }, STOW)

            .state(SCORE_SAMP)
            .onEnter { speedDecrease = 0.0; turnDecrease = 0.0 }
            .transitionTimed(SCORE_WAIT, STOW)
            .transition({ currentGamepad2.x && !previousGamepad2.x }, STOW)
            .build()
        var lastState = STOW

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        waitForStart()
        samplerFSM.start()
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            // drive
            // TODO: disable during PTO
            drivetrain.setEffort(
                -sign(gamepad1.left_stick_y.toDouble()) * gamepad1.left_stick_y.toDouble().pow(2) / (1 + speedDecrease),
                -sign(gamepad1.left_stick_x.toDouble()) * gamepad1.left_stick_x.toDouble().pow(2) / (1 + speedDecrease),
                -sign(gamepad1.right_stick_x.toDouble()) * gamepad1.right_stick_x.toDouble().pow(2) / (1.5 + turnDecrease),
            )

            // lift
            lift.read()
            if (currentGamepad2.a && !previousGamepad2.a) { lift.setPreset(Lift.Preset.BOTTOM) ; speedDecrease = 0.0 }
            if (currentGamepad2.b && !previousGamepad2.b && sampHeights) { lift.setPreset(Lift.Preset.SAMP_LOW) ; speedDecrease = 2.0 }
            if (currentGamepad2.y && !previousGamepad2.y && sampHeights) { lift.setPreset(Lift.Preset.SAMP_HIGH) ; speedDecrease = 2.0 }
            if (currentGamepad2.b && !previousGamepad2.b && specHeights) { lift.setPreset(Lift.Preset.SPEC_LOW) ; speedDecrease = 2.0 }
            if (currentGamepad2.y && !previousGamepad2.y && specHeights) { lift.setPreset(Lift.Preset.SPEC_HIGH) ; speedDecrease = 2.0 }

            if (
                currentGamepad2.left_bumper && currentGamepad2.right_bumper &&
                currentGamepad2.dpad_up && !previousGamepad2.dpad_up
            ) {
                sampler.incrementPitch()
            }
            if (
                currentGamepad2.left_bumper && currentGamepad2.right_bumper &&
                currentGamepad2.dpad_down && !previousGamepad2.dpad_down
            ) {
                sampler.decrementPitch()
            }
            if (currentGamepad2.start && !previousGamepad2.start) { manual = !manual }
            if (!manual) {
            } else {
                lift.setEffort(-gamepad2.left_stick_y + 0.2)
                if (gamepad2.dpad_down && -gamepad2.right_stick_y < -0.8) {
                    lift.resetEncoder()
                }
            }

            lastState = samplerFSM.state as Sampler.State
            samplerFSM.update()
            if (samplerFSM.state != lastState) {
                sampler.setState(samplerFSM.state as Sampler.State)
            }
            // safety in other states
            when {
                currentGamepad1.right_trigger > 0.8 && previousGamepad1.right_trigger <= 0.8 -> { sampler.setRoll(R90) }
                currentGamepad1.left_bumper && !previousGamepad1.left_bumper -> { sampler.setRoll(R45) }
                currentGamepad1.right_bumper && !previousGamepad1.right_bumper -> { sampler.setRoll(RCW45) }
            }
            sampler.updateProfiled()

            drivetrain.write()
            lift.write()
            sampler.write()

            telemetry.addLine("regular pathway - g1 left trigger")
            telemetry.addLine("alt pathway - g1 right trigger")
            telemetry.addLine("lift ctrls, samp & spec - g2 a, b, y")
            telemetry.addLine("reset to stow - g2 x")
            telemetry.addLine("manual lift - g2 start")
            telemetry.addLine("lift reset (in manual) - g2 dpad up + left stick up")
            telemetry.addLine("               ")
            telemetry.addData("height", lift.getHeight())
            telemetry.addData("state", samplerFSM.state)
            telemetry.update()
        }
    }
}
