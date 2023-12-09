package com.team2898.robot.commands.autos

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import com.team2898.robot.Constants
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase

class TestAuto : CommandBase() {
    val path = PathPlannerPath.fromPathFile("Test Auto")
    init {
        AutoBuilder.followPathWithEvents(path)
    }
}