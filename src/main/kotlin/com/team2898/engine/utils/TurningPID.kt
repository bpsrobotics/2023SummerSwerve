package com.team2898.engine.utils

import edu.wpi.first.wpilibj.Timer
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign

class TurningPID(val kP: Double, val kD: Double) {
    var setPoint = 0.0
    var timePrevious = Timer.getFPGATimestamp()
    var previousError = 0.0

    companion object {
        fun minCircleDist(anglea: Double, angleb: Double): Double{
            val normal = angleb - anglea
            val wrap = -((2 * PI) * normal.sign - normal)
            println("normal: $normal wrap: $wrap")
            val error = if (normal.absoluteValue < wrap.absoluteValue) {
                normal
            } else{
                wrap
            }
            return error
        }
    }

    fun motorOutput(sensorValue: Double): Double {
        val timeNow = Timer.getFPGATimestamp()


        val timeDif = timeNow - timePrevious
        val error = minCircleDist(sensorValue, setPoint)
        val errorDif = error - previousError

        val derivative = kD * ((errorDif) / (timeDif))


        previousError = error
        timePrevious = Timer.getFPGATimestamp()
        return error * kP + derivative
    }
}

fun main() {
//    println(TurningPID.minCircleDist(0.)
}
