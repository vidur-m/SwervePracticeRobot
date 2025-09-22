// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private DriveSubsystem m_driveSub;

  private UsbCamera camera1;
  private UsbCamera camera2;
  private NetworkTableEntry cameraSelection;
  private VideoSink server;
  private boolean cameraSource;

  private final CommandXboxController m_driverController =
  new CommandXboxController(OIConstants.k_DriverControllerPort);

  public RobotContainer() {
    initSubsystems();    
   // Configure the trigger bindings
   configureBindings();
 }
 
  private void initSubsystems() {
    m_driveSub = new DriveSubsystem();
    m_driveSub.setDefaultCommand(new RunCommand(
            () -> m_driveSub.drive(
                    OIConstants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisY), OIConstants.k_DriveDeadband), 
                    OIConstants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisX), OIConstants.k_DriveDeadband), 
                    OIConstants.k_driverRotAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisRot), OIConstants.k_DriveDeadband), 
                    true,
                    "Default / Field Oriented"
            ), 
            m_driveSub)
    );
  }

  private void configureBindings() {
    controllerPresetMain();
  }


  public void controllerPresetMain() {

    m_driverController.leftBumper().onTrue(
                        new InstantCommand(
                                () -> {
                                        m_driveSub.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
                                },
                                m_driveSub
                        )
    );
    
    if(OperatingConstants.k_usingSwerveDrive){
      m_driverController.leftTrigger().whileTrue(
              new RunCommand(
                      () -> m_driveSub.drive(
                              OIConstants.k_driverYAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisY), OIConstants.k_DriveDeadband), 
                              OIConstants.k_driverXAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisX), OIConstants.k_DriveDeadband), 
                              OIConstants.k_driverRotAxisInverted * MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.k_driverAxisRot), OIConstants.k_DriveDeadband), 
                              false,
                              "Robot Orientated"
                      ), 
                      m_driveSub
              )
      );
    }
  }
}
