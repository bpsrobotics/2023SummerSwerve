package com.team2898.robot.subsystems
import com.kauailabs.navx.frc.AHRS

/** Container object for an instantiated NavX class, as well as other functions relating to the gyroscope */
object NavX {
    /** Gyroscope used on robot */
    var navx = AHRS()
    /** @return The NavX's angle multiplied by -1 */
    fun getInvertedAngle(): Double{
        return -navx.angle
    }
}