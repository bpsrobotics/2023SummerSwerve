package com.team2898.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team2898.engine.utils.Sugar.degreesToRadians
import com.team2898.robot.Constants
import com.team2898.robot.OI
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase


object ToteGrabber : SubsystemBase(){
    // Limit switches
    // NEO 550 (Built in encoder)
    // Hit limit switch @ top
    // Dont hit limit switch at bottom
    // Go up / down
    private val armSparkMax = CANSparkMax(Constants.ToteGrabberConstants.kMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val limitSwitchTop = DigitalInput(Constants.ToteGrabberConstants.kToteLimitSwitchTop)
    private val limitSwitchBottom = DigitalInput(Constants.ToteGrabberConstants.kToteLimitSwitchBottom)
    private val armSparkMaxEncoder = armSparkMax.encoder
    private val armPidController = armSparkMax.pidController
    public var ToteGrab = false

    init {
        armPidController.p = Constants.ToteGrabberConstants.kArmP
        armSparkMax.restoreFactoryDefaults()
        armSparkMax.setSmartCurrentLimit(2)
    }

    override fun periodic() {
        var speedMultiplier = Constants.ToteGrabberConstants.kVolts
        if (armSparkMax.motorTemperature >= 50){
            speedMultiplier = 0.0
        }
        if(OI.grabTote){
            ToteGrab = true
        }
//        TurningPID.minCircleDist(armSparkMaxEncoder.position, desiredPosition)
        if (ToteGrab){

            if (limitSwitchBottom.get()) {
                speedMultiplier = 0.0
            }
            armSparkMax.set(1*speedMultiplier)
        } else {
            if (armSparkMaxEncoder.position == 90.degreesToRadians()) {
                speedMultiplier = 0.0
            }
            if (limitSwitchTop.get()) {
                speedMultiplier = 0.0
            }
            armSparkMax.set(-1*speedMultiplier)

        }
        SmartDashboard.putNumber("arm pos", armSparkMaxEncoder.position)
        SmartDashboard.putNumber("arm rate", 1*speedMultiplier)
        SmartDashboard.putBoolean("arm limit switch bottom", limitSwitchBottom.get())
        SmartDashboard.putBoolean("arm limit switch top", limitSwitchTop.get())
    }



}