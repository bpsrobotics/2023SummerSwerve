package com.team2898.robot.commands

//The commands for both the driver and operator
//import com.com.engine.utils.`M/s`
//import com.team2898.robot.Constants.ArmHeights.*
//import com.team2898.robot.Field
//import com.team2898.robot.OI
//import com.team2898.robot.subsystems.Arm

import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.engine.utils.Sugar.eqEpsilon
import com.team2898.engine.utils.TurningPID
import com.team2898.robot.OI
import com.team2898.robot.subsystems.Drivetrain
import com.team2898.robot.subsystems.NavX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

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
    var angle = 0.0;

    val PID = TurningPID(3.0,0.05)
    val kS = 0.1

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if(OI.turnX != 0.0 || OI.turnY !=0.0) {
            if(sqrt(OI.turnX.pow(2) + OI.turnY.pow(2)) > 0.75) angle = -atan2(OI.turnX, OI.turnY)
        }
        println(angle)
        SmartDashboard.putNumber("angle", angle)

        PID.setPoint = angle
        SmartDashboard.putNumber("power", PID.turnspeedOutput(-NavX.getInvertedAngle().degreesToRadians()))
        var turnSpeed= PID.turnspeedOutput(-NavX.getInvertedAngle().degreesToRadians())
        if(turnSpeed.eqEpsilon(0,0.05)) {}
        else if (turnSpeed < 0) { turnSpeed -= kS }
        else { turnSpeed += kS }
        Drivetrain.drive(-OI.translationY, -OI.translationX, turnSpeed, fieldRelative = true, rateLimit = true)
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
