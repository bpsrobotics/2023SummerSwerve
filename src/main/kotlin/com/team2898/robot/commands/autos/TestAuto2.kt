package com.team2898.robot.commands.autos

import com.team2898.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj2.command.CommandBase


class TestAuto2 : CommandBase() {
    private val time = edu.wpi.first.wpilibj.Timer()
    override fun execute(){
        println(time.get())
        if(!time.hasElapsed(3.0)) Drivetrain.drive(0.25, 0.0,0.0,true, true)
        else Drivetrain.drive(0.0,0.0,0.0,true,true)
    }

    override fun initialize() {
        time.reset()
        time.start()
    }
//    override fun end(interrupted: Boolean){
//        Drivetrain.drive(0.0,0.0,0.0,true,true)
//    }

    override fun isFinished(): Boolean {
        return false
    }
}