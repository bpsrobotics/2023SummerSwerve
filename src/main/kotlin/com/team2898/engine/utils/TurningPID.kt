package com.team2898.engine.utils

import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.engine.utils.Sugar.radiansToDegrees
import edu.wpi.first.wpilibj.Timer
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign

class TurningPID(var kP: Double, var kD: Double) {
    var setPoint = 0.0
    var timePrevious = Timer.getFPGATimestamp()
    var previousError = 0.0

    companion object {
        /** Takes in two radians and finds the lowest distance in radians **/
        fun minCircleDist(anglea: Double, angleb: Double): Double{
            val normal = angleb - anglea
            val wrap = -((2 * PI) * normal.sign - normal)
//            println("normal: ${normal} wrap: ${wrap} sign: ${normal.sign} anglea: $anglea angleb: $angleb")
            val circleDistance = if (normal.absoluteValue < wrap.absoluteValue) {
                normal
            } else{
                wrap
            }
            return circleDistance
        }
    }

    fun motorOutput(sensorValue: Double): Double {
        val timeNow = Timer.getFPGATimestamp()


        val timeDif = timeNow - timePrevious
        var useSetPoint = setPoint % (2 * PI)
        if(useSetPoint < 0) { useSetPoint += (2*PI)}
        val error = minCircleDist(sensorValue, setPoint)
        val errorDif = error - previousError

        val derivative = kD * ((errorDif) / (timeDif))


        previousError = error
        timePrevious = Timer.getFPGATimestamp()
        return error * kP + derivative
    }
}

//fun main() {
//    println(TurningPID.minCircleDist(0.52, 2 * PI))
//}
