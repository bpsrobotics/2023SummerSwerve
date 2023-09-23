package com.team2898.robot.subsystems

import com.team2898.robot.Constants
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry

object Odometry {
    var SwerveOdometry = SwerveDriveOdometry(
        Constants.DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(NavX.getInvertedAngle()), arrayOf(
            Drivetrain.m_frontLeft.position,
            Drivetrain.m_frontRight.position,
            Drivetrain.m_rearLeft.position,
            Drivetrain.m_rearRight.position
        ))
    fun update(){
        NavX.update()
        SwerveOdometry.update(
            Rotation2d.fromDegrees(NavX.getInvertedAngle()), arrayOf(
                Drivetrain.m_frontLeft.position,
                Drivetrain.m_frontRight.position,
                Drivetrain.m_rearLeft.position,
                Drivetrain.m_rearRight.position
            ))
    }
    @Suppress("unused")
    fun resetOdometry(pose: Pose2d?) {
        SwerveOdometry.resetPosition(
            Rotation2d.fromDegrees(NavX.getInvertedAngle()), arrayOf(
                Drivetrain.m_frontLeft.position,
                Drivetrain.m_frontRight.position,
                Drivetrain.m_rearLeft.position,
                Drivetrain.m_rearRight.position
            ),
            pose)
    }
}