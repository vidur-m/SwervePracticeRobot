package frc.robot.commands.SwerveDrivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TeleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveJoystickCommand extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final String statusName;

    /**
     * Contructs a Command to control the swerve via joystick
     * @param swerveSubsystem subsystem that controls the swerve
     * @param xSpdFunction x speed (left and right)
     * @param ySpdFunction y speed (forward and backwards)
     * @param turningSpdFunction turning speed (rotation) not angle control
     * @param fieldOrientedFunction field orientation (true for field orientated, false for robot orientated)
     * @param statusName the name of which command the robot is running
     */
    public SwerveJoystickCommand(DriveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, String statusName) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.statusName = statusName;
        this.xLimiter = new SlewRateLimiter(TeleConstants.k_MaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(TeleConstants.k_MaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(TeleConstants.k_MaxAngularSpeedRadiansPerSecondSquared);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Drive Mode", statusName); // Helps understand which command swerve drive is using
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = OIConstants.k_driverXAxisInverted * xSpdFunction.get();
        double ySpeed = OIConstants.k_driverYAxisInverted * ySpdFunction.get();
        double turningSpeed = OIConstants.k_driverRotAxisInverted * turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.k_DriveDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.k_DriveDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.k_DriveDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * TeleConstants.k_MaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * TeleConstants.k_MaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * TeleConstants.k_MaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.k_DriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        // Debuging
        if(DebuggingConstants.k_swerveDriveDebug) {
            SmartDashboard.putString("Joystick", "X : " + xSpdFunction.get() + "Y : " + ySpdFunction.get() + " Theta : " + turningSpdFunction.get());
            SmartDashboard.putString("ChassisSpeeds", "X : " + chassisSpeeds.vxMetersPerSecond + "Y : " + chassisSpeeds.vyMetersPerSecond + " Theta : " + chassisSpeeds.omegaRadiansPerSecond);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
