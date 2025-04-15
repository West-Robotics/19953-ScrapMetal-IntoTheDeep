package com.scrapmetal.util.hardware

import com.qualcomm.robotcore.hardware.Gamepad
import com.scrapmetal.util.control.deadzone
import kotlin.math.pow
import kotlin.math.sign
import kotlin.reflect.KProperty1
import kotlin.reflect.full.memberProperties

class SMGamepad(val gamepad: Gamepad) {
    val a       = Digital(Gamepad::a)
    val b       = Digital(Gamepad::b)
    val x       = Digital(Gamepad::x)
    val y       = Digital(Gamepad::y)
    val up      = Digital(Gamepad::dpad_up)
    val down    = Digital(Gamepad::dpad_down)
    val left    = Digital(Gamepad::dpad_left)
    val right   = Digital(Gamepad::dpad_right)
    val lb      = Digital(Gamepad::left_bumper)
    val rb      = Digital(Gamepad::right_bumper)
    val lsb     = Digital(Gamepad::left_stick_button)
    val rsb     = Digital(Gamepad::right_stick_button)
    val back    = Digital(Gamepad::back)
    val start   = Digital(Gamepad::start)
    val guide   = Digital(Gamepad::guide)

    val lsx     = Analog(Gamepad::left_stick_x)
    val lsy     = Analog(Gamepad::left_stick_y)
    val rsx     = Analog(Gamepad::right_stick_x)
    val rsy     = Analog(Gamepad::right_stick_y)
    val lt      = Analog(Gamepad::left_trigger)
    val rt      = Analog(Gamepad::right_trigger)

    fun update() {
        SMGamepad::class.memberProperties.forEach {
            if (it(this) !is Gamepad) {
                (Input::update)(it(this) as Input, gamepad)
            }
        }
    }

    class Digital(val button: KProperty1<Gamepad, Boolean>) : Input {
        private var state = false
        private var previous = state

        val pressed get() = state
        val rising get() = !previous && state
        val falling get() = previous && !state

        override fun update(gamepad: Gamepad) {
            previous = state
            state = button(gamepad)
        }
    }

    class Analog(val axis: KProperty1<Gamepad, Float>, val threshold: Double = 0.8) : Input {
        private var state = 0.0
        private var previous = state

        // val pos get() = state.deadzone(min = 0.05, max = 1.0, deadzone = 0.05)
        val pos get() = state
        val sq get() = sign(state) * state.pow(2)
        val rising get() = previous < threshold && threshold <= state
        val falling get() = previous >= threshold && threshold > state

        override fun update(gamepad: Gamepad) {
            previous = state
            state = axis(gamepad).toDouble()
        }
    }

    interface Input {
        fun update(gamepad: Gamepad)
    }
}