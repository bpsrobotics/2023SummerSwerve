// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot.subsystems

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
import com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxAbsoluteEncoder.Type
import com.revrobotics.SparkMaxPIDController
import com.team2898.robot.Constants.ModuleConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState

class MAXSwerveModule(drivingCANId: Int, turningCANId: Int, chassisAngularOffset: Double) {
    private val m_drivingSparkMax: CANSparkMax
    private val m_turningSparkMax: CANSparkMax
    private val m_drivingEncoder: RelativeEncoder
    private val m_turningEncoder: AbsoluteEncoder
    private val m_drivingPIDController: SparkMaxPIDController
    private val m_turningPIDController: SparkMaxPIDController
    private var m_chassisAngularOffset = 0.0
    private var m_desiredState = SwerveModuleState(0.0, Rotation2d())

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    init {
        m_drivingSparkMax = CANSparkMax(drivingCANId, kBrushless)
        m_turningSparkMax = CANSparkMax(turningCANId, kBrushed)

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_drivingSparkMax.restoreFactoryDefaults()
        m_turningSparkMax.restoreFactoryDefaults()

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder()
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle)
        m_drivingPIDController = m_drivingSparkMax.pidController
        m_turningPIDController = m_turningSparkMax.pidController
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder)
        m_turningPIDController.setFeedbackDevice(m_turningEncoder)

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.positionConversionFactor = ModuleConstants.kDrivingEncoderPositionFactor
        m_drivingEncoder.velocityConversionFactor = ModuleConstants.kDrivingEncoderVelocityFactor

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted)

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turningPIDController.positionPIDWrappingEnabled = true
        m_turningPIDController.positionPIDWrappingMinInput = ModuleConstants.kTurningEncoderPositionPIDMinInput
        m_turningPIDController.positionPIDWrappingMaxInput = ModuleConstants.kTurningEncoderPositionPIDMaxInput

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_drivingPIDController.p = ModuleConstants.kDrivingP
        m_drivingPIDController.i = ModuleConstants.kDrivingI
        m_drivingPIDController.d = ModuleConstants.kDrivingD
        m_drivingPIDController.ff = ModuleConstants.kDrivingFF
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
            ModuleConstants.kDrivingMaxOutput)

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_turningPIDController.p = ModuleConstants.kTurningP
        m_turningPIDController.i = ModuleConstants.kTurningI
        m_turningPIDController.d = ModuleConstants.kTurningD
        m_turningPIDController.ff = ModuleConstants.kTurningFF
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
            ModuleConstants.kTurningMaxOutput)
        m_drivingSparkMax.idleMode = ModuleConstants.kDrivingMotorIdleMode
        m_turningSparkMax.idleMode = ModuleConstants.kTurningMotorIdleMode
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.burnFlash()
        m_turningSparkMax.burnFlash()
        m_chassisAngularOffset = chassisAngularOffset
        m_desiredState.angle = Rotation2d(m_turningEncoder.getPosition())
        m_drivingEncoder.position = 0.0
    }

    /** The current state of the module. */
    val state: SwerveModuleState
        get() =// Apply chassis angular offset to the encoder position to get the position
            // relative to the chassis.
            SwerveModuleState(m_drivingEncoder.velocity,
                Rotation2d(m_turningEncoder.position - m_chassisAngularOffset))
    /** The current position of the module. */
    val position: SwerveModulePosition
        get() =// Apply chassis angular offset to the encoder position to get the position
            // relative to the chassis.
            SwerveModulePosition(
                m_drivingEncoder.position,
                Rotation2d(m_turningEncoder.position - m_chassisAngularOffset))

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState(desiredState: SwerveModuleState) {
        // Apply chassis angular offset to the desired state.
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))

        // Optimize the reference state to avoid spinning further than 90 degrees.
        val optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            Rotation2d(m_turningEncoder.position))

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity)
        m_turningPIDController.setReference(optimizedDesiredState.angle.radians, CANSparkMax.ControlType.kPosition)
        m_desiredState = desiredState
    }

    /** Zeroes all the SwerveModule encoders.  */
    fun resetEncoders() {
        m_drivingEncoder.position = 0.0
    }
}