package org.firstinspires.ftc.teamcode.experimental

import com.scrapmetal.util.architecture.*
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pControl
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.channels.Channel.Factory.CONFLATED
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import org.firstinspires.ftc.teamcode.experimental.subsystem.Claw
import org.firstinspires.ftc.teamcode.experimental.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.experimental.subsystem.Lift
import kotlin.math.abs

class Tasks(
    private val state: RobotState,
    private val pipelines: Pipelines,
    private val drivetrain: Drivetrain,
    private val lift: Lift,
    private val claw: Claw,
) {
    suspend fun runLiftTo(ref: Double) = coroutineScope {
        action(pipelines.lift) {
            while (abs(ref - state.height.value) < 5.0) {
                lift.setEffort(pControl(0.01, ref, state.height.value))
            }
        }
    }

    suspend fun driveByGamepad() = coroutineScope {
        action(pipelines.drivetrain) {
            while (isActive) {
                drivetrain.drive(Vector2d(state.forward.value, state.turn.value))
            }
        }
    }

    suspend fun forwardByTime(time: Long) = coroutineScope {
        action(pipelines.drivetrain) {
            drivetrain.drive(Vector2d(0.5, 0.0))
            delay(time)
            drivetrain.drive(Vector2d(0.0, 0.0))
        }
    }

    suspend fun scorePreload() = coroutineScope {
        action {
            claw.setState(Claw.State.GRAB)
            delay(100L)
            val rlt = runLiftTo(12.0)
            rlt.start()
            delay(250L)
            val fbt = forwardByTime(4000L)
            fbt.start()
            rlt.join()
            fbt.join()
            claw.setState(Claw.State.OPEN)
            runLiftTo(0.0)
        }
    }
}

// i don't think this can be an object expression because of weird lifetime stuff
class Pipelines {
    val drivetrain = Pipeline()
    val lift = Pipeline()
    val claw = Pipeline()
}