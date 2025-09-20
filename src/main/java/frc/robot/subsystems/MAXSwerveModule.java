package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;
import frc.robot.Constants.DriveConstants.MotorLocation;

public class MAXSwerveModule {
    private final SparkMax m_driveMotor;
    private final SparkMax m_turnMotor;
  
    private final RelativeEncoder m_driveEncoder;
    private final AbsoluteEncoder m_turnEncoder;
  
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;
  
    private final MotorLocation m_motorLocation;
    private final double m_driveEncoderInverted;
  
    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwerveModule(int p_drivingCANId, int p_turningCANId, double p_chassisAngularOffset, boolean p_driveEncoderInverted, SparkMaxConfig drivingConfig, MotorLocation p_motorLocation) {

        m_driveMotor = new SparkMax(p_drivingCANId, MotorType.kBrushless);
        m_turnMotor = new SparkMax(p_turningCANId, MotorType.kBrushless);
    
        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getAbsoluteEncoder();
    
        m_drivingClosedLoopController = m_driveMotor.getClosedLoopController();
        m_turningClosedLoopController = m_turnMotor.getClosedLoopController();
    
        m_motorLocation = p_motorLocation;
        m_driveMotor.configure(drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        m_turnMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        m_chassisAngularOffset = p_chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
        if(p_driveEncoderInverted) {
        m_driveEncoderInverted = -1.0;
        } else {
        m_driveEncoderInverted = 1.0;
        }
        m_driveEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(
          getDriveVelocity(),
            new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));
    }

    public double getDrivePosition() {
        return m_driveEncoder.getPosition() * m_driveEncoderInverted;
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    public double getTurnPosition() {
        return m_turnEncoder.getPosition();
    }

    public double getTurnVelocity() {
        return m_turnEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
          getDrivePosition(),
            new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        correctedDesiredState.optimize(new Rotation2d(m_turnEncoder.getPosition()));
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }

    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
    }
    
    public void stopMotors() {
        m_driveMotor.stopMotor();
        m_turnMotor.stopMotor();
    }
}