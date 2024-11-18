package com.scrapmetal.util.architecture
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.*
import kotlinx.coroutines.channels.Channel.Factory.CONFLATED
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

fun CoroutineScope.action(block: suspend CoroutineScope.() -> Unit): Action
    = launch(start = CoroutineStart.LAZY, block = block)

fun CoroutineScope.action(pipeline: Pipeline, block: suspend CoroutineScope.() -> Unit): Action
    = launch(start = CoroutineStart.LAZY) {
        pipeline.send(action(block))
      }

class Pipeline {
    private val channel = Channel<Action>(CONFLATED)

    suspend fun send(action: Action) = channel.send(action)

    /**
     * Begins pipeline, will never finish and needs to be called in a launch
     */
    suspend fun start() = coroutineScope {
        var currentAction: Action = launch { }
        for (action in channel) {
            currentAction.cancelAndJoin()
            currentAction = action
            currentAction.start()
        }
    }
}

typealias Action = Job