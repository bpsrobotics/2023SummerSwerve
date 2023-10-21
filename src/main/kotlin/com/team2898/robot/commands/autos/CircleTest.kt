package com.team2898.robot.commands.autos

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import com.team2898.engine.utils.TurningPID
import com.team2898.robot.subsystems.NavX
import com.team2898.robot.subsystems.Odometry
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase

class CircleTest : CommandBase() {
    private lateinit var autoCommandGroup: Command

    override fun initialize() {
        PathPlanner.loadPath("circular", PathConstraints(2.0, 2.0))
//        PPSwerveControllerCommand("circular", Odometry.pose.x, Odometry.pose.y, TurningPID, )
//        PPSwerveControllerCommand("TestPath", pose)
    }
}