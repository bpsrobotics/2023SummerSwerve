package com.team2898.robot.commands.autos

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import com.team2898.robot.Constants
import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class TestAuto : CommandBase() {
    private lateinit var autoCommandGroup: Command
    val path = PathPlannerPath.fromPathFile("Test Auto")
    override fun initialize() {
        autoCommandGroup = SequentialCommandGroup (
            AutoBuilder.followPathWithEvents(path)
        )
        autoCommandGroup.schedule()
    }

    override fun isFinished(): Boolean {
        return false
    }
}