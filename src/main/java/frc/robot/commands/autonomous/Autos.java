package frc.robot.commands.autonomous;

import java.io.ObjectInputFilter.Config;
import java.util.List;
import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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
            false, // If alliance flipping should be enabled 
            swerveSubsystem // The drive subsystem
        );
    }

    // private boolean isRed() {
    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    // }

    // public Command simpleAuto() {
    //     return Commands.sequence(
    //         autoFactory.trajectoryCmd("Simple auto")
    //     );
    // }

    // public AutoRoutine noneAuto() {
    //     return autoFactory.newRoutine("None");
    // }

    // public AutoRoutine simpleAuto() {
    //     StructPublisher<Pose2d> poseStartPublisher = NetworkTableInstance.getDefault().getStructTopic("AUTO POSE START", Pose2d.struct).publish();
    //     StructPublisher<Pose2d> poseEndPublisher = NetworkTableInstance.getDefault().getStructTopic("AUTO POSE END", Pose2d.struct).publish();
    //     AutoRoutine routine = autoFactory.newRoutine("center");

    //     // Load the routine's trajectories
    //     AutoTrajectory driveToMiddle = routine.trajectory("Simple auto");

        
    //     poseStartPublisher.set(driveToMiddle.getInitialPose().get());
    //     poseEndPublisher.set(driveToMiddle.getFinalPose().get());

    //     // When the routine begins, reset odometry and start the first trajectory (1)
    //     routine.active().onTrue(
    //         Commands.sequence(
    //             driveToMiddle.resetOdometry(),
    //             driveToMiddle.cmd()
    //         )
    //     );


    //     // driveToMiddle.done().onTrue();
    //     return routine;
    // }

    // public AutoRoutine auto2() {
    //     AutoRoutine routine = autoFactory.newRoutine("auto2");

    //     // Load the routine's trajectories
    //     AutoTrajectory driveToMiddle = routine.trajectory("auto2");

    //     // When the routine begins, reset odometry and start the first trajectory (1)
    //     routine.active().onTrue(
    //         Commands.sequence(
    //             driveToMiddle.resetOdometry(),
    //             driveToMiddle.cmd(),
    //             new Shoot(intake)
    //         )
    //     );
    //     driveToMiddle.done();
    //     return routine;
    // }
    
}
