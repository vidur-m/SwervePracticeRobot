package frc.robot.commands.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class MoveDirection extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final String statusName;
    private final Translation2d translation;
    private Pose2d finalPosition;
    private double xVal, yVal;
    private final double multiplier, threshold;

    /**
     * Contructs a Command to control the swerve via joystick
     * @param swerveSubsystem subsystem that controls the swerve
     * @param xSpdFunction x speed (left and right)
     * @param ySpdFunction y speed (forward and backwards)
     * @param turningSpdFunction turning speed (rotation) not angle control
     * @param fieldOrientedFunction field orientation (true for field orientated, false for robot orientated)
     * @param statusName the name of which command the robot is running
     */

     public MoveDirection(DriveSubsystem swerveSubsystem, String statusName, Translation2d translation)
     {
        this.swerveSubsystem = swerveSubsystem;
        this.statusName = statusName;
        this.translation = translation;
        multiplier = 0.75;
        threshold = 0.2;
        addRequirements(swerveSubsystem);
     }

     @Override
     public void initialize() 
     {
        SmartDashboard.putString("Drive Mode", "MoveDirection:" + statusName);
        finalPosition = swerveSubsystem().getPose().plus(new Transform2d(translation, swerveSubsystem.getRotation2d()));
        double distance = Math.sqrt(translation.getX() * translation.getX() + translation.getY() * translation.getY());
        xVal = multiplier * translation.getX() / distance;
        yVal = multiplier * translation.getY() / distance;
     }

     @Override
     public void execute()
     {
        SmartDashboard.putString("Joystick", "X : " + xVal + " Y : " + yVal + "Theta : " + 0);
        swerveSubsystem.drive(xVal, yVal, 0, false, "MoveDirection: " + statusName);
     }

     @Override
     public void end(boolean interrupted) 
     {
         swerveSubsystem.stopModules();
     }

     @Override
     public boolean isFinished()
     {
        Pose2d distancePose = swerveSubsystem.getPose().relativeTo(finalPosition);
        double distance = Math.sqrt(distancePose.getX() * distancePose.getX() + distancePose.getY() * distancePose.getY());
        return distance < threshold;
     }
}