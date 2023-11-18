package com.team2898.robot.subsystems
import com.kauailabs.navx.frc.AHRS
import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.engine.utils.Sugar.radiansToDegrees
import com.team2898.engine.utils.TurningPID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/** Container object for an instantiated NavX class, as well as other functions relating to the gyroscope */
object NavX {
    /** Gyroscope used on robot */
    var navx = AHRS()
    var totalRotation = 0.0;
    private var lastRotation = 0.0;
    var rotationalSpeed = 0.0;
    /** @return The NavX's angle multiplied by -1 */
    fun getInvertedAngle(): Double{
        return -navx.angle
    }
    fun update(timeSinceUpdate: Double){
        totalRotation += TurningPID.minCircleDist(navx.angle.degreesToRadians(), lastRotation.degreesToRadians()).radiansToDegrees()
        rotationalSpeed = TurningPID.minCircleDist(navx.angle.degreesToRadians(), lastRotation.degreesToRadians()).radiansToDegrees()/timeSinceUpdate
        lastRotation = navx.angle
        SmartDashboard.putNumber("TotalRotation", totalRotation)
    }
    fun reset(){
        navx.reset()
        totalRotation = 0.0;
    }
}