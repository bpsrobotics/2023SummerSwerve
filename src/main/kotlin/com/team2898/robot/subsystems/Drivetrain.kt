// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team2898.robot.subsystems

import com.team2898.engine.utils.SwerveUtils
import com.team2898.robot.Constants.DriveConstants
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Drivetrain
    : SubsystemBase() {

    // Create MAXSwerveModules
    private val m_frontLeft: MAXSwerveModule = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset)
    private val m_frontRight: MAXSwerveModule = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset)
    private val m_rearLeft: MAXSwerveModule = MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset)
    private val m_rearRight: MAXSwerveModule = MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset)

    // The gyro sensor
    private val m_gyro = ADIS16470_IMU()

    // Slew rate filter variables for controlling lateral acceleration
    private var m_currentRotation = 0.0
    private var m_currentTranslationDir = 0.0
    private var m_currentTranslationMag = 0.0
    private val m_magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
    private val m_rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
    private var m_prevTime = WPIUtilJNI.now() * 1e-6

    // Odometry class for tracking robot pose
    var m_odometry = SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(m_gyro.angle), arrayOf(
            m_frontLeft.position,
            m_frontRight.position,
            m_rearLeft.position,
            m_rearRight.position
    ))

    override fun periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(m_gyro.angle), arrayOf(
                m_frontLeft.position,
                m_frontRight.position,
                m_rearLeft.position,
                m_rearRight.position
        ))
        SmartDashboard.putNumber("Encoders/FL Turning Encoder", m_frontLeft.m_turningEncoder.position)
        SmartDashboard.putNumber("Encoders/FR Turning Encoder", m_frontRight.m_turningEncoder.position)
        SmartDashboard.putNumber("Encoders/BR Turning Encoder", m_rearRight.m_turningEncoder.position)
        SmartDashboard.putNumber("Encoders/BL Turning Encoder", m_rearLeft.m_turningEncoder.position)
    }

    /** Current estimated pose of the robot.*/
    val pose: Pose2d
        get() = m_odometry.poseMeters

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    fun resetOdometry(pose: Pose2d?) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(m_gyro.angle), arrayOf(
                m_frontLeft.position,
                m_frontRight.position,
                m_rearLeft.position,
                m_rearRight.position
        ),
                pose)
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     * field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean, rateLimit: Boolean) {
        val xSpeedCommanded: Double
        val ySpeedCommanded: Double
        if (rateLimit) {
            // Convert XY to polar for rate limiting
            val inputTranslationDir = Math.atan2(ySpeed, xSpeed)
            val inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2.0) + Math.pow(ySpeed, 2.0))

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            val directionSlewRate: Double
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag)
            } else {
                directionSlewRate = 500.0 //some high number that means the slew rate is effectively instantaneous
            }
            val currentTime = WPIUtilJNI.now() * 1e-6
            val elapsedTime = currentTime - m_prevTime
            val angleDif: Double = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir)
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime)
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag)
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0)
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI)
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag)
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime)
                m_currentTranslationMag = m_magLimiter.calculate(0.0)
            }
            m_prevTime = currentTime
            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir)
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir)
            m_currentRotation = m_rotLimiter.calculate(rot)
        } else {
            xSpeedCommanded = xSpeed
            ySpeedCommanded = ySpeed
            m_currentRotation = rot
        }

        // Convert the commanded speeds into the correct units for the drivetrain

        val xSpeedDelivered: Double = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        val ySpeedDelivered: Double = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        val rotDelivered: Double = m_currentRotation * DriveConstants.kMaxAngularSpeed
        val swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.angle)) else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered))
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond)
        m_frontLeft.setDesiredState(swerveModuleStates.get(0))
        m_frontRight.setDesiredState(swerveModuleStates.get(1))
        m_rearLeft.setDesiredState(swerveModuleStates.get(2))
        m_rearRight.setDesiredState(swerveModuleStates.get(3))
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    fun lock() {
        m_frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
        m_frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        m_rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        m_rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond)
        m_frontLeft.setDesiredState(desiredStates[0])
        m_frontRight.setDesiredState(desiredStates[1])
        m_rearLeft.setDesiredState(desiredStates[2])
        m_rearRight.setDesiredState(desiredStates[3])
    }

    /** Resets the drive encoders to currently read a position of 0.  */
    fun resetEncoders() {
        m_frontLeft.resetEncoders()
        m_rearLeft.resetEncoders()
        m_frontRight.resetEncoders()
        m_rearRight.resetEncoders()
    }

    /** Zeroes the heading of the robot.  */
    fun zeroHeading() {
        m_gyro.reset()
    }
    /** The robot's heading in degrees, from -180 to 180 */
    val heading: Double
        get() = Rotation2d.fromDegrees(m_gyro.angle).degrees
    /** The turn rate of the robot, in degrees per second */
    val turnRate: Double
        get() = m_gyro.rate * if (DriveConstants.kGyroReversed) -1.0 else 1.0
}