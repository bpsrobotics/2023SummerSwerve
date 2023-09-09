package com.team2898.robot.commands

//The commands for both the driver and operator
//import com.com.engine.utils.`M/s`
//import com.team2898.robot.Constants.ArmHeights.*
//import com.team2898.robot.Field
//import com.team2898.robot.OI
//import com.team2898.robot.subsystems.Arm

import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.MAXSwerveModule
import com.team2898.robot.subsystems.NavX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase

/**
    Called when the Tele-Operated stage of the game begins.
 */
class TeleOp : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }
    // Called when the command is started.
    override fun initialize() {
        //SmartDashboard.putNumber("goal", PI)
        Drivetrain.zeroHeading()

    }
    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        Drivetrain.drive(-OI.translationY, -OI.translationX, -OI.turnX, fieldRelative = true, rateLimit = true)
        SmartDashboard.putNumber("NavX", NavX.getInvertedAngle())

    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }


    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
