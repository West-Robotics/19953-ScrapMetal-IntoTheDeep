# Ninth Wheel
FTC Team 19953 Scrap Metal's code for our Into The Deep robot, Ninth Wheel

## Overview
The code uses structured concurrency via [`kotlinx.coroutines`](https://kotlinlang.org/docs/coroutines-guide.html). Robot tasks are often natural to express synchronously (e.g. while loops), which can then be implemented asynchronously using coroutines so that multiple tasks can run at once. Please read [Coroutine basics](https://kotlinlang.org/docs/coroutines-basics.html) before continuing.

## Hardware
Each hardware call in FTC blocks for ~2 ms[^1], so it makes things more convenient to have all hardware access happen in one coroutine.

A typical hardware access coroutine might look like
```kotlin
launch {
    while (opModeIsActive()) {
        hub.clearBulkCache()
        // internally reads sensors and updates StateFlows
        state.update()
        // write stored inputs to hardware
        drivetrain.write()
        lift.write()
    }
}
```

[^1]: See [Eeshwar's blog](https://blog.eeshwark.com/robotblog/photonftc-basic-explanation) for more info

## Subsystems
A subsystem is a class that only exists to conveniently command ≥ 1 actuators that work together. The actuators of a subsystem should not be tightly coupled to (its existence should not physically depend on nor vice versa) the actuators of another subsystem. All of its actuators should be properties of the class. The class should not contain any looping functions. It should provide a single function (in rare cases multiple) that receives a desired actuation as a parameter but only stores it and does not write to hardware. The function may process the actuation before writing, like applying inverse kinematics or translating from an enum state to servo positions, but should not involve any control loop logic like feedforwards or slewing. The function name should reflect the action taken: for example, `drive()` would indicate taking some sort of direction vector, `setPower()` would indicate explicitly setting motor power, and `setState()` would indicate setting some higher-level state. A subsystem must implement the Subsystem (TODO: link) interface, which requires the class to implement a `write()` function that writes the stored actuation to hardware.

Example claw subsystem class:
```kotlin
class Claw(hardwareMap: HardwareMap) : Subsystem {
    // in our actual code we use our own servo wrappers
    private val pinch = hardwareMap.servo.get("pinch")
    private val wrist = hardwareMap.servo.get("wrist")
    private var pinchPos = State.STOW.pinch
    private var wristPos = State.STOW.wrist

    init {
        pinch.setDirection(Servo.Direction.REVERSE)
        wrist.setDirection(Servo.Direction.FORWARD)
    }

    fun setState(state: State) {
        pinchPos = state.pinch
        wristPos = state.wrist
    }

    enum class State(val pinch: Double, val wrist: Double) {
        OPEN(1.0, 1.0),
        GRAB(0.0, 1.0),
        STOW(0.0, 0.0), 
    }

    override fun write() {
        pinch.setPosition(pinchPos)
        wrist.setPosition(wristPos)
    }
}
```

## Robot State
The class RobotState (TODO: link) holds information gathered by sensors, calculated from sensor data, received from gamepads, etc. that can then be used for control decisions by other coroutines. The data is exposed through a set of [StateFlows](https://kotlinlang.org/api/kotlinx.coroutines/kotlinx-coroutines-core/kotlinx.coroutines.flow/-state-flow/), a type of [Flow](https://kotlinlang.org/docs/flow.html). StateFlows emit a single value which other coroutines can collect whenever it is updated. They are _hot flows_ which always run regardless of whether it is being collected, unlike most Flows which are _cold_.

Example RobotState usage:
```kotlin
// in reality we would probably use an action here as described under Task Execution
launch {
    // this will never complete and the lambda will execute on each new update
    state.height.collect { height -> updatePid(height) }
}

launch {
    // collectLatest() gets the most recent value from the flow and returns immediately
    while (state.height.collectLatest { height -> abs(height-endpoint) > 0.01 }) {
        followMotionProfile()
    }
}
```

## Task Execution
### Actions
An action is a coroutine that actuates the robot in a specific way and usually ends when the actuation is done (unless the action is maintaining a PID that always runs, etc.). An action may be composed of subactions, and actions are enclosed in a top-level task. Both actions and tasks are implemented as a [Job](https://kotlinlang.org/api/kotlinx.coroutines/kotlinx-coroutines-core/kotlinx.coroutines/-job/), which is a reference to a coroutine. Actions are built using the `action()` (TODO: link) builder, which is a wrapper around the `launch()` builder with a pipeline parameter. The action will immediately be sent to the given pipeline. Tasks are a simple wrapper around `launch()` with the lazy start parameter so that it only begins when triggered by `start()` or `join()`).

<!-- timeout cancellation and cooperative cancellation for stopped opmode: behavior appears to differ??? either don't need, can try/finally, or while isActive -->
<!-- maybe make top-level action functions not lazy so you don't need join? or is that more confusing? -->

Example action:
```kotlin
// extend intake and raise lift, then retract intake and lower lift
// this function returns a lazy Job
fun unjam() = task {
    // create a new CoroutineScope: this action will not proceed until each of this subscope's constituent coroutines finish
    // however, it can still suspend as it is under a different, suspending coroutine's scope
    coroutineScope {
        // this action is immediately sent to the intake's pipeline, which will decide when to actually run it
        action(pipelines.intake) { intake.setState(Intake.State.EXTEND) }
        action(pipelines.lift) {
            while (state.height.collectLatest { height -> abs(height-10.0) > 0.01 }) {
                // assume the pid is always being run by another coroutine
                setPidReference(10.0)
            }
        }
    }
    // wait for 1 sec
    delay(1000L)
    // assume the stow action is already defined somewhere
    // join() will wait for the job to complete before allowing unjam to finish
    stow().join()
}

unjam().join()
```

### Pipelines
A pipeline is a medium through which a subsystem can receive and execute an action. Each subsystem has its own corresponding pipeline. Pipelines receive actions through [Channels](https://kotlinlang.org/docs/channels.html), another form of inter-coroutine communication where values can be sent to the channel by multiple coroutines and those values can be received by other channels. All Channels in pipelines are *conflated*, meaning they can hold at most one action at a time, and sending more to the channel will drop the oldest ones. This serves as a mechanism to both never have stale actions, and to deal with multiple actions requiring the same subsystems.

A subsystem should only ever obey one action at a time so that it doesn't fight itself. Actions actuate different sets of subsystems, so some actions may be able to run in parallel with others (e.g. an action on the drivetrain and an action on the lift), but some may cause conflict (e.g. an action on the drivetrain and intake and an action on just the drivetrain). Additionally, if one schedules one conflicting action after another, the expectation is that the first action gets cancelled immediately instead of waiting for completion or ignoring the second action. For example, the operator could schedule an intake extension, realize that it was a bad idea because it's going to collide with the wall, and schedule a retraction to immediately reverse the extension. Instead of a subsystem dependency manager, individual pipelines defend their own subsystems, which percolates upwards. As an example, say that a previously empty pipeline receives an action (which is only a subaction of a higher-level action) in a channel (which empties the channel) and starts the action. The pipeline monitors the channel and sees that a second action has been sent and the first action is still running. It will cancel the first action and start the second action. However, cancelling the first action will also cancel its parent action and its parent's parent and so on, ensuring that the entire task is removed instead of leaving some parts of the task running. This means no special case needs to be handled between conflicting actions, since if any conflict occurs, the original action will be cancelled, and if there is no conflict, then nothing special happens.

<!-- it's good to have the inputs only loosely coupled to actions so that you can directly set inputs if needed for debugging etc. instead of always needing actions to do anything -->
<!-- and loose coupling between inputs and hardware -->
<!-- consistent verbage and nouns? -->
<!-- exceptions for timeouts, auto errors, etc. -->

## OpModes
`runOpMode()` must be defined using the `runBlocking()` builder to create a blocking coroutine scope.

### Autonomous
Autonomous opmodes have a similar setup to usual, except the routines are specified by chaining coroutines instead. More complicated routines and error-handling can still be created with this system.

In general, it is convenient to have all tasks be defined within a class for dependency injection and a single stored location. If specifying the instance of the task class is too annoying, scope functions can be used like `with()`.

Example:
```kotlin
tasks.hardwareAccess().start()
waitUntilStart()
coroutineScope {
    // start this action and let it continue in the background
    tasks.driveToLocation1().start()
    delay(3500L)
    // deposit and wait until it's done
    tasks.deposit().join()
    // this scope will not finish until driveToLocation1() is done
}
coroutineScope {
    tasks.stow().start()
    tasks.driveToLocation2().join()
}
```

### Teleop
Running actions requested by gamepad is as simple as having a looping coroutine with a bunch of if statements.

Example:
```kotlin
launch {
    while (opModeIsActive()) {
        // assume buttons are rising edge
        if (gamepad.a) tasks.intake().start()
        if (gamepad.leftBumper) tasks.stow().start()
        if (gamepad.rightTrigger > 0.5) tasks.score().start()
    }
}

action(Action.DRIVETRAIN) {
    while (opModeIsActive()) {
        drivetrain.drive(Pose2d(gamepad.leftX, gamepad.leftY, -gamepad.rightY))
    }
}
```

Note that if some action interrupts the teleop driving action (for example auto-driving to a location), there needs to be a way to reschedule teleop driving.

## Diagram
Dotted lines means that the nodes are not directly controlled by each other.
<picture>
  <img alt="Shows a diagram describing an example of the concurrency architecture" src="/doc/media/concurrency-architecture.svg">
</picture>
