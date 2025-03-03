package frc.robot.commands.autonomous;

import java.io.ObjectInputFilter.Config;
import java.util.List;
import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralIntake;
// Pass in robot states


public class Autos {
    private final TrajectoryConfig trajectoryConfig;
    private final SwerveSubsystem swerveSubsystem = RobotContainer.swerveSubsystem;
    private final CoralIntake intake = RobotContainer.coralIntakeSubsystem;
    private final AutoFactory autoFactory;

    
    public Autos(){
        // 1. Create trajectory settings
        this.trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        autoFactory = new AutoFactory(
            swerveSubsystem::getPose, // A function that returns the current robot pose
            swerveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerveSubsystem // The drive subsystem
        );
    }

    public AutoRoutine noneAuto() {
        return autoFactory.newRoutine("None");
    }

    public AutoRoutine simpleAuto() {
        AutoRoutine routine = autoFactory.newRoutine("center");

        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine.trajectory("Simple auto");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                driveToMiddle.cmd()
            )
        );
        driveToMiddle.done().onTrue(new Shoot(intake));
        return routine;
    }
    
}
