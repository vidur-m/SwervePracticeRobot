package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.function.DoubleSupplier;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.Constants.DriveConstants.MotorLocation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.k_FrontLeftDrivingCanId,
        DriveConstants.k_FrontLeftTurningCanId,
        DriveConstants.k_FrontLeftChassisAngularOffset,
        DriveConstants.k_FrontLeftInverted,
        Configs.MAXSwerveModule.frontLeftDrivingConfig,
        MotorLocation.FRONT_LEFT);
    
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.k_FrontRightDrivingCanId,
        DriveConstants.k_FrontRightTurningCanId,
        DriveConstants.k_FrontRightChassisAngularOffset,
        DriveConstants.k_FrontRightInverted,
        Configs.MAXSwerveModule.frontRightDrivingConfig,
        MotorLocation.FRONT_RIGHT);
  
    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        DriveConstants.k_RearLeftDrivingCanId,
        DriveConstants.k_RearLeftTurningCanId,
        DriveConstants.k_BackLeftChassisAngularOffset,
        DriveConstants.k_RearLeftftInverted,
        Configs.MAXSwerveModule.rearLeftDrivingConfig,
        MotorLocation.REAR_LEFT);
  
    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
        DriveConstants.k_RearRightDrivingCanId,
        DriveConstants.k_RearRightTurningCanId,
        DriveConstants.k_BackRightChassisAngularOffset,
        DriveConstants.k_RearRightInverted,
        Configs.MAXSwerveModule.rearRightDrivingConfig,
        MotorLocation.REAR_RIGHT);

    private final Pigeon2 m_gyro = OperatingConstants.k_usingGyro ? new Pigeon2(DriveConstants.k_pigeon2Id) : null;

    private SwerveModuleState m_desiredModuleStates[] = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    private StructArrayPublisher<SwerveModuleState> publisherDesiredStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> publisherActualStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyActualStates", SwerveModuleState.struct).publish();

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.k_DriveKinematics,
        getRotation2d(),
        getSwerveModulePosition());
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        double multipler = 1;
        double alt = 1;
        drive(-robotRelativeSpeeds.vxMetersPerSecond/DriveConstants.k_MaxSpeedMetersPerSecond * multipler, -robotRelativeSpeeds.vyMetersPerSecond/DriveConstants.k_MaxSpeedMetersPerSecond * multipler, -robotRelativeSpeeds.omegaRadiansPerSecond/DriveConstants.k_MaxAngularSpeed * alt, false, "AutoBuilder");
    }
        

    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(), 
            m_frontRight.getState(), 
            m_rearLeft.getState(), 
            m_rearRight.getState()
        };
    }
    
    public SwerveModulePosition[] getSwerveModulePosition() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            getRotation2d(),
            getSwerveModulePosition(),
            pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, String statusName) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double multipler = 0.5;
        double xSpeedDelivered = xSpeed * DriveConstants.k_MaxSpeedMetersPerSecond * multipler;
        double ySpeedDelivered = ySpeed * DriveConstants.k_MaxSpeedMetersPerSecond * multipler;
        double rotDelivered = rot * DriveConstants.k_MaxAngularSpeed * multipler;
    
        var swerveModuleStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.k_MaxSpeedMetersPerSecond);
        m_desiredModuleStates = swerveModuleStates;
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    
        SmartDashboard.putString("Drive Mode", statusName); // Helps understand which command swerve drive is using
    }

    public void setX() {
        m_desiredModuleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        m_desiredModuleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        m_desiredModuleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        m_desiredModuleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    
        m_frontLeft.setDesiredState(m_desiredModuleStates[0]);
        m_frontRight.setDesiredState(m_desiredModuleStates[1]);
        m_rearLeft.setDesiredState(m_desiredModuleStates[2]);
        m_rearRight.setDesiredState(m_desiredModuleStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.k_MaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
        m_desiredModuleStates = desiredStates;
    }

    public void stopModules() {
        m_frontLeft.stopMotors();
        m_frontRight.stopMotors();
        m_rearLeft.stopMotors();
        m_frontRight.stopMotors();
    
        m_desiredModuleStates[0].speedMetersPerSecond = 0;
        m_desiredModuleStates[1].speedMetersPerSecond = 0;
        m_desiredModuleStates[2].speedMetersPerSecond = 0;
        m_desiredModuleStates[3].speedMetersPerSecond = 0;
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.k_DriveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        );
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(0);
    }

    public DoubleSupplier[] getWheelRotationSupplier() {
        return new DoubleSupplier[]{
            () -> m_frontLeft.getTurnPosition(),
            () -> m_frontRight.getTurnPosition(),
            () -> m_rearLeft.getTurnPosition(),
            () -> m_rearRight.getTurnPosition()
        };
    }
}
