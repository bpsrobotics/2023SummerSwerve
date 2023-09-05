package com.team2898.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.pow
import kotlin.math.sign

/**
 * The Operating Interface object.
 * This is where you put joystick, button, or keyboard inputs.
 *
 * A note about delegated properties, which are used in this object:
 *  A delegated property is where getting (or setting) a field is outsourced
 *  to another object.  Then, whenever you read the property, it asks the
 *  object the property is delegated to for the value.
 */
@Suppress("unused")
object OI : SubsystemBase() {
    /**
     * Threshold below which [process] will return 0.
     * 0.1 historically used, but optimal value unknown.
     */
    private const val DEADZONE_THRESHOLD = 0.1

    /**
     * Utility function for controller axis, optional deadzone and square/cube for extra fine-grain control
     */
    private fun process(
        input: Double,
        deadzone: Boolean = false,
        square: Boolean = false,
        cube: Boolean = false
    ): Double {
        var output = 0.0

        if (deadzone) {
            output = MathUtil.applyDeadband(input, DEADZONE_THRESHOLD)
        }

        if (square) {
            // To keep the signage for output, we multiply by sign(output). This keeps negative inputs resulting in negative outputs.
            output = output.pow(2) * sign(output)
        }

        if (cube) {
            // Because cubing is an odd number of multiplications, we don't need to multiply by sign(output) here.
            output = output.pow(3)
        }

        return output
    }

    // conflicts with the other definition, name it something else after compilation
    @JvmName("process1")
    fun Double.process(deadzone: Boolean = false, square: Boolean = false, cube: Boolean = false) =
        process(this, deadzone, square, cube)

    private val driverController = XboxController(0)
    private val operatorController = Joystick(1)

    // Left and right shoulder switches (the ones next to the trigger) for quickturn
    val quickTurnRight
        get() = process(driverController.rightTriggerAxis, deadzone = true, square = true)
    val quickTurnLeft
        get() = process(driverController.leftTriggerAxis, deadzone = true, square = true)

    // Right joystick y-axis.  Controller mapping can be tricky, the best way is to use the driver station to see what buttons and axis are being pressed.
    // Squared for better control on turn, cubed on throttle
    /** Driver controller's throttle on the left joystick for the X Axis, from -1 (left) to 1 (right) */
    val translationX
        get() = -process(driverController.leftX, deadzone = true, square = true)

    /** Driver controller's throttle on the left joystick for the Y Axis, from -1 (down) to 1 (up) */
    val translationY
        get() = process(-driverController.leftY, deadzone = true, square = true)

    /** Driver controller's throttle on the right joystick for the X Axis, from -1 (left) to 1 (right) */
    val turnX
        get() = -process(driverController.rightX, deadzone = true, square = false)
    /** Driver controller's throttle on the right joystick for the Y Axis, from -1 (down) to 1 (up) */
    val turnY
        get() = -process(driverController.rightY, deadzone = true, square = false)


    val highHat get() = operatorController.pov
    val moving get() = operatorController.getRawButton(7)

    enum class Direction {
        LEFT, RIGHT, UP, DOWN, INACTIVE;

        fun mirrored() = when (this) {
            LEFT  -> RIGHT
            RIGHT -> LEFT
            else  -> this
        }
    }

    val alignmentPad get() = when(driverController.pov) {
        0    -> Direction.UP
        90   -> Direction.RIGHT
        180  -> Direction.DOWN
        270  -> Direction.LEFT
        else -> Direction.INACTIVE
    }

    val operatorTrigger get() = operatorController.trigger


//    init {
//        Trigger { operatorController.pov != 0 }.toggleOnTrue(
//            Commands.startEnd(
//                Drivetrain::brakeMode,
//                Drivetrain::coastMode
//            )
//        )
//
//    }
}
