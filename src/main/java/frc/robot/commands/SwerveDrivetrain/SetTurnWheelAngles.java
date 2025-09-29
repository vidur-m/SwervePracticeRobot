package frc.robot.commands.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SetTurnWheelAngles extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final double angle;

    /**
     * Contructs a Command to control the swerve via joystick
     * @param swerveSubsystem subsystem that controls the swerve
     * @param xSpdFunction x speed (left and right)
     * @param ySpdFunction y speed (forward and backwards)
     * @param turningSpdFunction turning speed (rotation) not angle control
     * @param fieldOrientedFunction field orientation (true for field orientated, false for robot orientated)
     * @param statusName the name of which command the robot is running
     */
    public SetTurnWheelAngles(DriveSubsystem swerveSubsystem, double angle) {
        this.swerveSubsystem = swerveSubsystem;
        this.angle = angle;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Drive Mode", "TurnWheels/" + angle); // Helps understand which command swerve drive is using
        SwerveModuleState[] moduleStates = new SwerveModuleState[4]; 
        moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
        moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
        moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
        moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}