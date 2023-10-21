package com.team2898.robot.commands.autos

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase

class TestAuto : CommandBase() {
    private lateinit var autoCommandGroup: Command

    override fun initialize() {
        PathPlanner.loadPath("TestPath", PathConstraints(2.0, 2.0))
//        PPSwerveControllerCommand("TestPath", pose)
    }
}