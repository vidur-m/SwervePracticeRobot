// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private DriveSubsystem m_driveSub;

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
