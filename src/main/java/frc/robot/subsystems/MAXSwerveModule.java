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

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int p_drivingCANId, int p_turningCANId, double p_chassisAngularOffset, boolean p_driveEncoderInverted, SparkMaxConfig drivingConfig, MotorLocation p_motorLocation) {

    m_driveMotor = new SparkMax(p_drivingCANId, MotorType.kBrushless);
    m_turnMotor = new SparkMax(p_turningCANId, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_driveMotor.getClosedLoopController();
    m_turningClosedLoopController = m_turnMotor.getClosedLoopController();

    m_motorLocation = p_motorLocation;

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
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

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
      getDriveVelocity(),
        new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
     * Gets the encoder position of the turn motor in radians
     * @return double of turn motor's encoder value converted
     */
    public double getDrivePosition() {
      return m_driveEncoderInverted * m_driveEncoder.getPosition();
  }

  /**
   * Gets this module's drive velocity in meters / second
   * @return double of velocity of the drive module converted
   */
  public double getDriveVelocity() {
      return m_driveEncoder.getVelocity();
  }

    /**
     * Gets the encoder position of the turn motor in radians
     * @return double of turn motor's encoder value converted
     */
    public double getTurnPosition() {
        return m_turnEncoder.getPosition();
    }

    /**
     * Gets this module's turn velocity in meters / second
     * @return double of velocity of the turn module converted
     */
    public double getTurnVelocity() {
        return m_turnEncoder.getVelocity();
    }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
      getDrivePosition(),
        new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turnEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }

  public void stopMotors() {
    m_driveMotor.stopMotor();
    m_turnMotor.stopMotor();
  }
}
