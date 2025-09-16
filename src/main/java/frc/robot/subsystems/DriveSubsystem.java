package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.Constants.TeleConstants;
import frc.robot.Constants.DriveConstants.MotorLocation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.k_FrontLeftDriveCanId,
        DriveConstants.k_FrontLeftTurningCanId,
        DriveConstants.k_FrontLeftChassisAngularOffset,
        DriveConstants.k_FrontLeftInverted,
        Configs.MAXSwerveModule.frontLeftDrivingConfig,
        MotorLocation.FRONT_LEFT);
    
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.k_FrontRightDriveCanId,
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
    
}
