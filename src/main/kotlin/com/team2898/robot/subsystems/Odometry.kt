package com.team2898.robot.subsystems

import com.team2898.robot.Constants
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Odometry {
    var SwerveOdometry = SwerveDriveOdometry(
        Constants.DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(NavX.getInvertedAngle()), arrayOf(
            Drivetrain.m_frontLeft.position,
            Drivetrain.m_frontRight.position,
            Drivetrain.m_rearLeft.position,
            Drivetrain.m_rearRight.position
        ))
    val pose: Pose2d
        get() = SwerveOdometry.poseMeters

    /** Robot rotation speed in m/s */
    var velocity: Translation2d = Translation2d()
    private var lastPose = Pose2d()
    private val timer = Timer();
    init {
        timer.start()
    }
    fun update(){
        NavX.update(timer.get())
        SwerveOdometry.update(
            Rotation2d.fromDegrees(NavX.getInvertedAngle()), arrayOf(
                Drivetrain.m_frontLeft.position,
                Drivetrain.m_frontRight.position,
                Drivetrain.m_rearLeft.position,
                Drivetrain.m_rearRight.position
            ))
        velocity = Translation2d((lastPose.x - pose.x)/timer.get(), (lastPose.y - pose.y)/timer.get())
        lastPose = pose
        SmartDashboard.putNumber("Odometry/FieldX", pose.x)
        SmartDashboard.putNumber("Odometry/FieldY", pose.y)
        SmartDashboard.putNumberArray("Odometry/velocity", arrayOf(velocity.x,velocity.y))
        SmartDashboard.putNumber("Odometry/test", timer.get())
        timer.reset()
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