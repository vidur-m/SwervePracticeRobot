package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ModuleConstants;

public class Configs {
    public static final class MAXSwerveModule {

        public static final SparkMaxConfig frontRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rearRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rearLeftDrivingConfig = new SparkMaxConfig();

        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();


        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double d_drivingFactor = ModuleConstants.k_WheelDiameterMeters * Math.PI
                    / ModuleConstants.k_DrivingMotorReduction;
            double d_turningFactor = 2 * Math.PI;
            double d_drivingVelocityFeedForward = 1 / ModuleConstants.k_DriveWheelFreeSpeedRps;

            frontRightDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            frontRightDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            frontRightDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to (change?) them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            frontLeftDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            frontLeftDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            frontLeftDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            rearRightDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            rearRightDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            rearRightDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            rearLeftDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            rearLeftDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            rearLeftDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(d_turningFactor) // radians
                    .velocityConversionFactor(d_turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, d_turningFactor); 
        }
    }
}
