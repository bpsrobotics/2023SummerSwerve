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
import com.team2898.robot.subsystems.Odometry
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import kotlin.math.pow
import kotlin.math.sign
enum class DriveMode {
    Normal,
    Defense
}
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
        breakTimer.start()

    }

    var angle = 0.0;

    val PID = TurningPID(3.5,0.05)
    val kS = 0.1
    val breakTimer = Timer()
    var breakTimerGoal = 0.0
    var drivemode = DriveMode.Normal
    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if(OI.defenseModeButton) drivemode = DriveMode.Defense
        if(OI.normalModeButton) drivemode = DriveMode.Normal
        if(drivemode == DriveMode.Normal) {
            Drivetrain.drive(
                -(OI.translationY), //* OI.translationY.sign,
                -(OI.translationX), //* OI.translationX.sign,
                OI.turnX,
                fieldRelative = true,
                rateLimit = true
            )


            /*if(OI.turnX != 0.0 || OI.turnY !=0.0) {
            if(sqrt(OI.turnX.pow(2) + OI.turnY.pow(2)) > 0.75) angle = -atan2(OI.turnX, OI.turnY)
        }*/
        }
        else if(drivemode == DriveMode.Defense) {
            angle += OI.turnX.pow(2).degreesToRadians() * -5 * OI.turnX.sign
            SmartDashboard.putNumber("angle", angle)

            PID.setPoint = angle
            println(NavX.totalRotation)
            var turnSpeed = PID.turnspeedOutputNoNormalize(-NavX.totalRotation.degreesToRadians())
            if (turnSpeed.eqEpsilon(0, 0.04)) {
            } else if (turnSpeed < 0) {
                turnSpeed -= kS
            } else {
                turnSpeed += kS
            }
            if (OI.translationX == 0.0 && OI.translationY == 0.0 && turnSpeed.eqEpsilon(0, 0.04)) {
                if (breakTimer.hasElapsed(breakTimerGoal)) Drivetrain.lock()
                else Drivetrain.drive(0.0, 0.0, 0.0, fieldRelative = true, rateLimit = true)
            } else {
                Drivetrain.drive(
                    -(OI.translationY), //* OI.translationY.sign,
                    -(OI.translationX), //* OI.translationX.sign,
                    turnSpeed,
                    fieldRelative = true,
                    rateLimit = true
                )
                breakTimer.reset()
                breakTimerGoal = (Odometry.velocity.norm + turnSpeed / 2) / 3
            }
            SmartDashboard.putNumber("NavX", NavX.getInvertedAngle())
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }


    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
