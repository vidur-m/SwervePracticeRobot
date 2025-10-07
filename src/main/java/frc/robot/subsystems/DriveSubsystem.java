package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
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

/* Drive Subsystem */
public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
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

  // The gyro sensor
  private final Pigeon2 m_gyro = OperatingConstants.k_usingGyro ? new Pigeon2(DriveConstants.k_pigeon2Id) : null;
  // private final AHRS m_NAVXGyro = new AHRS(NavXComType.kMXP_SPI);

  private SwerveModuleState m_desiredModuleStates[] = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
  private StructArrayPublisher<SwerveModuleState> publisherDesiredStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> publisherActualStates = NetworkTableInstance.getDefault().getStructArrayTopic("MyActualStates", SwerveModuleState.struct).publish();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.k_DriveKinematics,
      getRotation2d(),
      getSwerveModulePosition());

  /** Creates a new DriveSubsystem. */

    /**
     * Need To Check If Still Need This
     * @param robotRelativeSpeeds
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
      double multipler = 1;
      double alt = 1;
      drive(-robotRelativeSpeeds.vxMetersPerSecond/DriveConstants.k_MaxSpeedMetersPerSecond * multipler, -robotRelativeSpeeds.vyMetersPerSecond/DriveConstants.k_MaxSpeedMetersPerSecond * multipler, -robotRelativeSpeeds.omegaRadiansPerSecond/DriveConstants.k_MaxAngularSpeed * alt, false, "AutoBuilder");
      
      /*
       // Stripped from Template Pathplanner Github
      ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

      SwerveModuleState[] targetStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(targetSpeeds);
      setModuleStates(targetStates);
       */
    }
    /**
     * Gets a list of Swerve Module States
     * @return A list of Swerve Module State from front left, front right, back left, back right
     */
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

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      getRotation2d(),
        getSwerveModulePosition());

        if(DebuggingConstants.k_swerveDriveDebug) {
            updateSmartDashboard();
            // updateWheelPositions();
            
            publisherDesiredStates.set(m_desiredModuleStates);
            publisherActualStates.set(getSwerveModuleState());
        }
  }

  /**
     * Updates general robot data to SmartDasboard such as heading or pose
     */
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading (Yaw)", getRotation2d().getDegrees());
        if(OperatingConstants.k_usingGyro) {
          SmartDashboard.putNumber("Roll", m_gyro.getRoll().getValue().in(Units.Degree));
          SmartDashboard.putNumber("Pitch", m_gyro.getPitch().getValue().in(Units.Degree));
        }
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    /**
     * Upddates wheel positions to help advantage scope and other tools to visualize robot
     */
    public void updateWheelPositions() {
        SwerveModuleState[] moudleStates = getSwerveModuleState(); // Current states of wheels
        
        // Physical / IRL values
        SmartDashboard.putNumberArray(
            "RealState"
            , new double[]{
                moudleStates[0].angle.getRadians(),
                moudleStates[0].speedMetersPerSecond,
                moudleStates[1].angle.getRadians(),
                moudleStates[1].speedMetersPerSecond,
                moudleStates[2].angle.getRadians(),
                moudleStates[2].speedMetersPerSecond,
                moudleStates[3].angle.getRadians(),
                moudleStates[3].speedMetersPerSecond,
            }
        );

        // Controller / Desired values
        SmartDashboard.putNumberArray(
            "DesiredState"
            , new double[]{
                m_desiredModuleStates[0].angle.getRadians(),
                m_desiredModuleStates[0].speedMetersPerSecond,
                m_desiredModuleStates[1].angle.getRadians(),
                m_desiredModuleStates[1].speedMetersPerSecond,
                m_desiredModuleStates[2].angle.getRadians(),
                m_desiredModuleStates[2].speedMetersPerSecond,
                m_desiredModuleStates[3].angle.getRadians(),
                m_desiredModuleStates[3].speedMetersPerSecond,
            }
        );

        // Phsyical / IRL Values but with Fake Speeds to help align and see deviation
        SmartDashboard.putNumberArray(
            "RealStateFakeSpeed"
            , new double[]{
                moudleStates[0].angle.getRadians(),
                5,
                moudleStates[1].angle.getRadians(),
                5,
                moudleStates[2].angle.getRadians(),
                5,
                moudleStates[3].angle.getRadians(),
                5,
            }
        );

        // Controller / Desired Values but with Fake Speeds to help align and see deviation
        SmartDashboard.putNumberArray(
            "DesiredStateFakeSpeed"
            , new double[]{
                m_desiredModuleStates[0].angle.getRadians(),
                4,
                m_desiredModuleStates[1].angle.getRadians(),
                4,
                m_desiredModuleStates[2].angle.getRadians(),
                4,
                m_desiredModuleStates[3].angle.getRadians(),
                4,
            }
        );
    }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getRotation2d(),
        getSwerveModulePosition(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field. (true for field orientated, false for robot orientated)
   */
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

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
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

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.k_MaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
    m_desiredModuleStates = desiredStates;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    if(OperatingConstants.k_usingGyro) {
      m_gyro.reset();
    }
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getRotation2d() {
    // return new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0)); // This way to avoid import issues
    if(OperatingConstants.k_usingGyro) {
      return new Rotation2d(m_gyro.getYaw().getValue());
    } else {
      return new Rotation2d(0);
    }
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

  /**
     * A function designed to only drive the wheels
     * @param speed Velocity (meters per second) to turn the wheels 
     */
    
    /**
     * A function designed to only turn the wheels
     * @param pos Position to turn wheels to */
    
    /**
     * A function designed to continously turn the wheels
     * @param wheelPos A supplier of what the newest wheel position is
     * @param turnClockwise Whether the wheel should turn clockwise or not
     */
    /**
     * A list of the swerve module's latest turn angle
     * @return A supplier of the wheels latest position
     */
    public DoubleSupplier[] getWheelRotationSupplier() {
        return new DoubleSupplier[]{
            () -> m_frontLeft.getTurnPosition(),
            () -> m_frontRight.getTurnPosition(),
            () -> m_rearLeft.getTurnPosition(),
            () -> m_rearRight.getTurnPosition()
        };
    }
}
