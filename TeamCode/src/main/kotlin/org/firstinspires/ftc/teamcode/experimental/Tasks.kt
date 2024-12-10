package org.firstinspires.ftc.teamcode.experimental

import com.scrapmetal.util.architecture.*
import com.scrapmetal.util.control.Vector2d
import com.scrapmetal.util.control.pControl
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.experimental.subsystem.Claw
import org.firstinspires.ftc.teamcode.experimental.subsystem.Drivetrain
import org.firstinspires.ftc.teamcode.experimental.subsystem.Lift
import kotlin.math.abs

class Tasks(
    private val scope: CoroutineScope,
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

    fun driveByGamepad() = scope.action(pipelines.drivetrain) {
        while (isActive) {
            println("Looping driveByGamepad")
            drivetrain.drive(Vector2d(state.forward.value, state.turn.value))
            yield()
        }
    }

    fun forwardByTime(time: Long) = scope.action(pipelines.drivetrain) {
        drivetrain.drive(Vector2d(0.5, 0.0))
        delay(time)
        drivetrain.drive(Vector2d(0.0, 0.0))
    }

    fun scorePreload() = scope.task {
        claw.setState(Claw.State.CLOSE)
        delay(1000L)
        // val rlt = runLiftTo(12.0)
        // rlt.start()
        // delay(250L)
        val fbt = forwardByTime(2000L)
        // fbt.start()
        // rlt.join()
        coroutineScope { fbt.join() }
        claw.setState(Claw.State.OPEN)
        // runLiftTo(0.0)
    }
}

// i don't think this can be an object expression because of weird lifetime stuff
class Pipelines {
    val drivetrain = Pipeline()
    val lift = Pipeline()
    val claw = Pipeline()
}