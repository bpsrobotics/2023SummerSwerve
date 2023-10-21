package com.team2898.robot.commands

//The commands for both the driver and operator
//import com.com.engine.utils.`M/s`
//import com.team2898.robot.Constants.ArmHeights.*
//import com.team2898.robot.Field
//import com.team2898.robot.OI
//import com.team2898.robot.subsystems.Arm

import com.team2898.engine.utils.Sugar.clamp
import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.NavX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import kotlin.math.*

class TeleOp : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }
    // Called when the command is started.
    override fun initialize() {
        SmartDashboard.putNumber("goal", PI)


    }
    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val yaw = NavX.navx.angle
        var driverAngle = atan2(OI.translationY, OI.translationX)

        if (!driverAngle.isFinite()){
            driverAngle = 0.0
        }
        val length = sqrt(OI.translationX.pow(2) + OI.translationY.pow(2))
        val newAngle = driverAngle - yaw
        val driveX = length * cos(newAngle)
        val driveY = length * sin(newAngle)

        Drivetrain.drive(driveX, driveY, OI.turnX, true, true)
//        Drivetrain.drive(OI.translationX, OI.translationY, OI.turnX, true, true)
        //Drivetrain.m_frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d(SmartDashboard.getNumber("goal", PI))))
        //Drivetrain.m_frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d(SmartDashboard.getNumber("goal", PI))))
        //Drivetrain.m_rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d(SmartDashboard.getNumber("goal", PI))))
        //Drivetrain.m_rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d(SmartDashboard.getNumber("goal", PI))))
        SmartDashboard.putNumber("throttleX", OI.translationX.clamp(-1,1))
        SmartDashboard.putNumber("throttleY", OI.translationY.clamp(-1,1))


    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }


    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
