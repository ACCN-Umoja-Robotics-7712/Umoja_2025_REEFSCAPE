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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoralArmStates;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralIntake;
// Pass in robot states


public class Autos {
    private final SwerveSubsystem swerveSubsystem = RobotContainer.swerveSubsystem;

    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(DriveConstants.kDriveKinematics);
    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, 0, AutoConstants.kThetaControllerConstraints);

    Pose2d blueReefRobotLeft = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackRight22, Constants.Measurements.branchOffset);
    Pose2d blueCenter = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackCenter21, Constants.Measurements.branchOffset);
    Pose2d blueReefRobotRight = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackLeft20, Constants.Measurements.branchOffset);
    Pose2d redReefRobotLeft = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackRight9, Constants.Measurements.branchOffset);
    Pose2d redCenter = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackCenter10, Constants.Measurements.branchOffset);
    Pose2d redReefRobotRight = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackLeft11, Constants.Measurements.branchOffset);
    Pose2d practiceCenter = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefLeft6, Constants.Measurements.branchOffset);
    
    Pose2d blueReefDriverLeftLeftBranch = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefLeft19, Constants.Measurements.branchOffset);
    Pose2d blueReefDriverLeftRightBranch = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefLeft19, -Constants.Measurements.branchOffset);
    Pose2d blueReefDriverRightLeftBranch = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefRight17, Constants.Measurements.branchOffset);
    Pose2d blueReefDriverRightRightBranch = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefRight17, -Constants.Measurements.branchOffset);

    Pose2d redReefDriverLeftLeftBranch = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefLeft6, Constants.Measurements.branchOffset);
    Pose2d redReefDriverLeftRightBranch = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefLeft6, -Constants.Measurements.branchOffset);
    Pose2d redReefDriverRightLeftBranch = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefRight8, Constants.Measurements.branchOffset);
    Pose2d redReefDriverRightRightBranch = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefRight8, -Constants.Measurements.branchOffset);

    Pose2d blueStationRobotLeft = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.bluePickupRight12, 0*Constants.Measurements.coralStationDivotOffset);
    Pose2d blueStationRobotRight = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.bluePickupLeft13,  -0*Constants.Measurements.coralStationDivotOffset);
    Pose2d redStationRobotLeft = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redPickupRight2,  0*Constants.Measurements.coralStationDivotOffset);
    Pose2d redStationRobotRight = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redPickupLeft1,  -0
    *Constants.Measurements.coralStationDivotOffset);
    
    public enum AUTO {
        BLUE_DRIVER_LEFT, BLUE_CENTER, BLUE_DRIVER_RIGHT,
        RED_DRIVER_LEFT, RED_CENTER, RED_DRIVER_RIGHT,
        PRACTICE_FIELD
    }
    private SendableChooser<AUTO> chooser;
    StructPublisher<Pose2d> reefPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Auto Reef Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> stationPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Auto Station Pose", Pose2d.struct).publish();
    
    public Autos(){
        // Create the auto chooser
        chooser = new SendableChooser<AUTO>();
    
        // Add options to the chooser
        chooser.addOption("Blue Driver Left", AUTO.BLUE_DRIVER_LEFT);
        chooser.addOption("Blue Center", AUTO.BLUE_CENTER);
        chooser.addOption("Blue Driver Right", AUTO.BLUE_DRIVER_RIGHT);
        chooser.addOption("Red Driver Left", AUTO.RED_DRIVER_LEFT);
        chooser.addOption("Red Center", AUTO.RED_CENTER);
        chooser.addOption("Red Driver Right", AUTO.RED_DRIVER_RIGHT);
        chooser.addOption("Practice field center", AUTO.PRACTICE_FIELD);
        chooser.setDefaultOption("Default", null);
        // // Put the auto chooser on the dashboard
        SmartDashboard.putData("AUTOS", chooser);
        
        // 1. Create trajectory settings
        this.trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        thetaController.enableContinuousInput(0, 360);
    }

    public Command getAuto() {
        AUTO auto = chooser.getSelected();
        return switch (auto) {
            case BLUE_DRIVER_LEFT -> getBlueDriverLeft();
            case BLUE_CENTER -> getBlueCenter();
            case BLUE_DRIVER_RIGHT -> getBlueDriverRight();
            case RED_DRIVER_LEFT -> getRedDriverLeft();
            case RED_CENTER -> getRedCenter();
            case RED_DRIVER_RIGHT -> getRedDriverRight();
            case PRACTICE_FIELD -> getPracticeField();
            default -> new InstantCommand();
        };
    }

    public Command getBlueDriverLeft() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), blueReefRobotRight, AutoConstants.firstWait),
            getStationCommand(blueReefRobotRight, blueStationRobotRight),
            getScoreCommand(swerveSubsystem.offsetPoint(blueStationRobotRight, 0, -1, 0), blueReefDriverLeftLeftBranch, AutoConstants.stationWait),
            getStationCommand(blueReefDriverLeftLeftBranch, blueStationRobotRight),
            getScoreCommand(swerveSubsystem.offsetPoint(blueStationRobotRight, 0, -1, 0), blueReefDriverLeftRightBranch, AutoConstants.stationWait)
        );
    }

    public Command getPracticeField() {
        return getScoreCommand(swerveSubsystem.getPose(), practiceCenter, AutoConstants.firstWait);
    }

    public Command getBlueCenter() {
        return getScoreCommand(swerveSubsystem.getPose(), blueCenter, AutoConstants.firstWait);
    }

    public Command getBlueDriverRight() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), blueReefRobotLeft, AutoConstants.firstWait),
            getStationCommand(blueReefRobotLeft, blueStationRobotLeft),
            getScoreCommand(swerveSubsystem.offsetPoint(blueStationRobotLeft, 0, -1, 0), blueReefDriverRightLeftBranch, AutoConstants.stationWait),
            getStationCommand(blueReefDriverRightLeftBranch, blueStationRobotLeft),
            getScoreCommand(swerveSubsystem.offsetPoint(blueStationRobotLeft, 0, -1, 0), blueReefDriverRightRightBranch, AutoConstants.stationWait)
        );
    }

    public Command getRedDriverLeft() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), redReefRobotRight, AutoConstants.firstWait),
            getStationCommand(redReefRobotRight, redStationRobotRight),
            getScoreCommand(swerveSubsystem.offsetPoint(redStationRobotRight, 0, -1, 0), redReefDriverLeftLeftBranch, AutoConstants.stationWait),
            getStationCommand(redReefDriverLeftLeftBranch, redStationRobotRight),
            getScoreCommand(swerveSubsystem.offsetPoint(redStationRobotRight, 0, -1, 0), redReefDriverLeftRightBranch, AutoConstants.stationWait)
        );
    }

    public Command getRedCenter() {
        return getScoreCommand(swerveSubsystem.getPose(), redCenter, AutoConstants.firstWait);
    }

    public Command getRedDriverRight() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), redReefRobotLeft, AutoConstants.firstWait),
            getStationCommand(redReefRobotLeft, redStationRobotLeft),
            getScoreCommand(swerveSubsystem.offsetPoint(redStationRobotLeft, 0, -1, 0), redReefDriverRightLeftBranch, AutoConstants.stationWait),
            getStationCommand(redReefDriverRightLeftBranch, redStationRobotLeft),
            getScoreCommand(swerveSubsystem.offsetPoint(redStationRobotLeft, 0, -1, 0), redReefDriverRightRightBranch, AutoConstants.stationWait)
        );
    }

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getScoreCommand(Pose2d startPose, Pose2d endPose, double waitTime) {
    // An example command will be run in autonomous

    if (endPose == null) {
      return new InstantCommand();
    }
    reefPosePublisher.set(endPose);
    
    Trajectory traj = TrajectoryGenerator.generateTrajectory(
      startPose,
      List.of(),
      endPose,
      trajectoryConfig);

    // 4. Construct command to follow trajectory 
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      traj,
      swerveSubsystem::getPose,
      DriveConstants.kDriveKinematics,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        // Drive then move elevator after 1 second
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules())
            ),
            new SequentialCommandGroup(
                new WaitCommand(waitTime),
                new ParallelCommandGroup(
                    new MoveElevator(RobotContainer.elevatorSubsystem, ElevatorStates.L4),
                    new MoveArm(RobotContainer.coralArmSubsystem, CoralArmStates.L4),
                    new Intake(RobotContainer.coralIntakeSubsystem)
                )
            )
        ),
        // keep intaking so if we hit branch, it'll center the coral
        new WaitCommand(0.3),
        // shoot for 1 second then stop
        new ParallelCommandGroup(
            new InstantCommand(() -> RobotContainer.coralIntakeSubsystem.runIntake(-1)),
            new WaitCommand(1.0)
        ),
        new InstantCommand(() -> RobotContainer.coralIntakeSubsystem.runIntake(0))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getStationCommand(Pose2d startPose, Pose2d endPose) {

    if (endPose == null) {
      return new InstantCommand();
    }
    stationPosePublisher.set(endPose);

    Trajectory traj = TrajectoryGenerator.generateTrajectory(
      swerveSubsystem.offsetPoint(startPose, 0, -1, 0),
      List.of(),
      swerveSubsystem.offsetPoint(endPose, 0, 0.5, 0),
      trajectoryConfig);

    // 4. Construct command to follow trajectory 
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        traj,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        // Move elevator then drive after 1.5 seconds
        new ParallelCommandGroup(
            new ParallelCommandGroup(
                new MoveElevator(RobotContainer.elevatorSubsystem, ElevatorStates.L1),
                new MoveArm(RobotContainer.coralArmSubsystem, CoralArmStates.PICKUP)
            ),
            new SequentialCommandGroup(
                new WaitCommand(1.1),
                new ParallelCommandGroup(
                    new Intake(RobotContainer.coralIntakeSubsystem),
                    swerveControllerCommand
                )
            )
        )
    );
  }
    
}
