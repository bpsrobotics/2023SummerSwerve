package com.team2898.engine.utils

import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.round

// Don't show warnings if these functions are unused or could be private
@Suppress("unused", "MemberVisibilityCanBePrivate")

/**
 * Sugar is an object where we are adding convenience functions
 */
object Sugar {

    /**
     * Converts the value from radians to degrees
     *
     * @return the value in radians as a double
     */
    fun Double.radiansToDegrees(): Double {
        return times(180 / PI)
    }
    /**
     * Returns true if the value is [maxDistance] from [target]
     * @param target The target value
     * @param maxDistance The greatest distance from the target value that will still return true
     * @author Ozy
     */
    fun Double.eqEpsilon(target: Double, maxDistance:Double = 0.01) = (this - target).absoluteValue < maxDistance
    /**
     * Returns true if the value is [maxDistance] from [target]
     * @param target The target value
     * @param maxDistance The greatest distance from the target value that will still return true
     * @author Ozy
     */
    fun Double.eqEpsilon(target: Int, maxDistance:Double = 0.01) = (this - target).absoluteValue < maxDistance
    /**
     * Returns true if the value is [maxDistance] from [target]
     * @param target The target value
     * @param maxDistance The greatest distance from the target value that will still return true
     * @author Ozy
     */
    fun Int.eqEpsilon(target: Double, maxDistance:Double = 0.01) = (this - target).absoluteValue < maxDistance
    /**
     * Returns true if the value is [maxDistance] from [target]
     * @param target The target value
     * @param maxDistance The greatest distance from the target value that will still return true
     * @author Ozy
     */
    fun Int.eqEpsilon(target: Int, maxDistance:Double = 0.01) = (this - target).absoluteValue < maxDistance


    /**
     * Converts the value from degrees to radians
     *
     * @return the value in degrees as a double
     */
    fun Double.degreesToRadians(): Double {
        return times(PI / 180)
    }


    /**
     * Converts the value from radians to degrees
     *
     * @return the value in radians as a double
     */
    fun Int.radiansToDegrees(): Double {
        return toDouble().radiansToDegrees()
    }

    /**
     * Converts the value from degrees to radians
     *
     * @return the value in degrees as a double
     */
    fun Int.degreesToRadians():Double{
        return toDouble().degreesToRadians()
    }

    /**
     * Clamps this double to be within the range [min]..[max], inclusive.
     */
    fun Double.clamp(min: Double = 0.0, max: Double = 1.0) = this.coerceIn(min, max)
    /**
     * Clamps this double to be within the range [min]..[max], inclusive.
     */
    fun Int.clamp(min: Int = 0, max: Int = 1) = this.coerceIn(min, max)

    /**
     * Convert an angle to an equivalent rotation between 0 and 2PI
     */
    fun Double.circleNormalize(): Double {
        if(this < 0) return (this % (2* PI)) + (2*PI)
        return this % (2*PI)
    }

    /**
     * Finds angle difference between two angles in radians
     */
    fun angleDifference(angle1: Double, angle2: Double): Double {
        val a = angle1 - angle2
        return (a + PI).mod(2.0 * PI) - PI
    }
    /**
     * Rounds the value to [decimalPlace] places
     */
    fun Double.roundTo(decimalPlace: Int) : Double{
        val multiplier = 10.0.pow(decimalPlace).toInt()
        return round(this*multiplier)/multiplier
    }
}

