// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxPIDController
import com.team2898.robot.Constants.ModuleConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.AnalogEncoder

class MAXSwerveModule(drivingCANId: Int, turningCANId: Int, turningEncoderID: Int, chassisAngularOffset: Double) {
    private val m_drivingSparkMax: CANSparkMax
    private val m_turningTalon: TalonSRX
    private val m_drivingEncoder: RelativeEncoder
    private val m_turningEncoder: AnalogEncoder
    private val m_drivingPIDController: SparkMaxPIDController
    private val feed_forward: SimpleMotorFeedforward
    private val m_turningPIDController: PIDController
    private var m_chassisAngularOffset = 0.0
    private var m_desiredState = SwerveModuleState(0.0, Rotation2d())

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    init {
        feed_forward = SimpleMotorFeedforward(0.0, 0.0)

        m_drivingSparkMax = CANSparkMax(drivingCANId, kBrushless)
        m_turningTalon = TalonSRX(turningCANId)

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_drivingSparkMax.restoreFactoryDefaults()
        m_turningTalon.configFactoryDefault()

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder()
        m_turningEncoder = AnalogEncoder(turningEncoderID)
        m_drivingPIDController = m_drivingSparkMax.pidController
        m_turningPIDController = PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder)

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.positionConversionFactor = ModuleConstants.kDrivingEncoderPositionFactor
        m_drivingEncoder.velocityConversionFactor = ModuleConstants.kDrivingEncoderVelocityFactor

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.distancePerRotation = ModuleConstants.kTurningEncoderPositionFactor

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
//        m_turningEncoder.rev(ModuleConstants.kTurningEncoderInverted)  //fixme

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
//        m_turningPIDController.positionPIDWrappingEnabled = true
//        m_turningPIDController.positionPIDWrappingMinInput = ModuleConstants.kTurningEncoderPositionPIDMinInput
//        m_turningPIDController.positionPIDWrappingMaxInput = ModuleConstants.kTurningEncoderPositionPIDMaxInput //fixme

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
        //m_turningPIDController.ff = ModuleConstants.kTurningFF
        //m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        //        ModuleConstants.kTurningMaxOutput)
        m_drivingSparkMax.idleMode = ModuleConstants.kDrivingMotorIdleMode
        m_turningTalon.setNeutralMode(NeutralMode.Brake)
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
        m_turningTalon.configContinuousCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.burnFlash()
        m_turningTalon.config_kP(0, ModuleConstants.kTurningP)
        m_turningTalon.config_kI(0, ModuleConstants.kTurningI)
        m_turningTalon.config_kD(0, ModuleConstants.kTurningD)

        m_chassisAngularOffset = chassisAngularOffset
        m_desiredState.angle = Rotation2d(m_turningEncoder.absolutePosition)
        m_drivingEncoder.position = 0.0
    }

    /** The current state of the module. */
    val state: SwerveModuleState
        get() =// Apply chassis angular offset to the encoder position to get the position
                // relative to the chassis.
            SwerveModuleState(m_drivingEncoder.velocity,
                    Rotation2d(m_turningEncoder.absolutePosition - m_chassisAngularOffset))
    /** The current position of the module. */
    val position: SwerveModulePosition
        get() =// Apply chassis angular offset to the encoder position to get the position
                // relative to the chassis.
            SwerveModulePosition(
                    m_drivingEncoder.position,
                    Rotation2d(m_turningEncoder.absolutePosition - m_chassisAngularOffset))

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
                Rotation2d(m_turningEncoder.absolutePosition))

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity)
        m_turningTalon.set(TalonSRXControlMode.Velocity, optimizedDesiredState.speedMetersPerSecond)
        //setReference(optimizedDesiredState.angle.radians, CANSparkMax.ControlType.kPosition)
        m_desiredState = desiredState
    }

    /** Zeroes all the SwerveModule encoders.  */
    fun resetEncoders() {
        m_drivingEncoder.position = 0.0
    }
}