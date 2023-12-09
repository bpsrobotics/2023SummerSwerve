package com.team2898.robot.subsystems


import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team2898.robot.Constants.DriveConstants.kFeddderCanId
import com.team2898.robot.Constants.DriveConstants.kFlywheelCanId
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Shooter : SubsystemBase() {

    private val flywheelController = CANSparkMax(kFlywheelCanId, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val feederController = CANSparkMax(kFeddderCanId, CANSparkMaxLowLevel.MotorType.kBrushless)

    val ready get() = (rawFlywheelSpeed - target.flywheel).value.absoluteValue < 10 &&
            (rawSpinnerSpeed - target.spinner).value.absoluteValue < 10 &&
            min(abs(rawFlywheelSpeed.value), abs(rawSpinnerSpeed.value)) > 10

    var target = ShooterSpeeds(0.RPM, 0.RPM)
    var shooterPower = ShooterPowers(0.0, 0.0)


    data class ShooterSpeeds(val flywheel: RPM, val spinner: RPM)
    data class ShooterPowers(val flywheel: Double, val spinner: Double)

    private var lastShooterPos = 0.0
    private var lastSpinnerPos = 0.0
    private val timer = Timer()
    init {
        timer.start()
    }

    m_drivingPIDController = m_drivingSparkMax.pidController

    private val shooterPID = flywheelController.pidController
    private val shooterFF = flywheelController.SimpleMotorFeedForward
    private val spinnerPID = feederController.pidController
    private val spinnerFF = feederController.SimpleMotorFeedForward

//    init {
//        spinnerEncoder.distancePerPulse = 1.0 / 256
//    }

    private val rawFlywheelSpeed get() = ((flywheelController.encoder.position - lastShooterPos) / timer.get() * 60).RPM
    private val rawSpinnerSpeed get() = ((spinnerController.encoder.position - lastSpinnerPos) / timer.get() * 60).RPM
    private val filteredShooterSpeed = MovingAverage(3)
    private val filteredSpinnerSpeed = MovingAverage(3)

    init {
        listOf(flywheelController, spinnerController).forEach {
            it.restoreFactoryDefaults()
            it.setSmartCurrentLimit(20)
            it.idleMode = CANSparkMax.IdleMode.kCoast
            it.inverted = true
        }

        flywheelController.pidController.ff = 0.0
        flywheelController.pidController.p  = 0.0
        flywheelController.pidController.i  = 0.0
        flywheelController.pidController.d  = 0.0
    }


}