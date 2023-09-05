@file:Suppress("unused", "FunctionName", "SpellCheckingInspection", "LocalVariableName")

package com.team2898.engine.utils

import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.sign

object SwerveUtils {
    /**
     * Steps a value towards a target with a specified step size.
     * @param _current The current or starting value.  Can be positive or negative.
     * @param _target The target value the algorithm will step towards.  Can be positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for `_current` after performing the specified step towards the specified target.
     */
    fun StepTowards(_current: Double, _target: Double, _stepsize: Double): Double {
        return if (abs(_current - _target) <= _stepsize) {
            _target
        } else if (_target < _current) {
            _current - _stepsize
        } else {
            _current + _stepsize
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param _current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param _target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for `_current` after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    fun StepTowardsCircular(_current: Double, _target: Double, _stepsize: Double): Double {
        //_current = _current
        //var _target = _target
        val wrappedCurrent = WrapAngle(_current)
        val wrappedTarget = WrapAngle(_target)
        val stepDirection = sign(wrappedTarget - wrappedCurrent)
        val difference = abs(wrappedCurrent - wrappedTarget)
        return when {
            difference <= _stepsize ->   wrappedTarget
            difference > Math.PI ->
                //does the system need to wrap over eventually?
                //handle the special case where you can reach the target in one step while also wrapping
                if (wrappedCurrent + 2 * Math.PI - wrappedTarget < _stepsize || wrappedTarget + 2 * Math.PI - wrappedCurrent < _stepsize) {
                    wrappedTarget
                } else {
                    WrapAngle(wrappedCurrent - stepDirection * _stepsize) //this will handle wrapping gracefully
                }
            else -> wrappedCurrent + stepDirection * _stepsize
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    fun AngleDifference(_angleA: Double, _angleB: Double): Double {
        val difference = abs(_angleA - _angleB)
        return if (difference > Math.PI) 2 * Math.PI - difference else difference
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    fun WrapAngle(_angle: Double): Double {
        val twoPi = 2 * Math.PI
        return if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            0.0
        } else if (_angle > twoPi) {
            _angle - twoPi * floor(_angle / twoPi)
        } else if (_angle < 0.0) {
            _angle + twoPi * (floor(-_angle / twoPi) + 1)
        } else {
            _angle
        }
    }
}