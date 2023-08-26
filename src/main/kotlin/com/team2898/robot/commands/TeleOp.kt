package com.team2898.robot.commands

//The commands for both the driver and operator
//import com.com.engine.utils.`M/s`
//import com.team2898.robot.Constants.ArmHeights.*
//import com.team2898.robot.Field
//import com.team2898.robot.OI
//import com.team2898.robot.subsystems.Arm

import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import kotlin.math.PI

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
        Drivetrain.drive(OI.throttleX, OI.throttleY, OI.turnX, true, true)
        //Drivetrain.m_frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d(SmartDashboard.getNumber("goal", PI))))

    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }


    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
