package frc.robot.commands.autonomous;

import java.io.ObjectInputFilter.Config;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.objects.Reef;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralIntake;

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

    Pose2d blueStationDriverRight = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.bluePickupRight12, 0*Constants.Measurements.coralStationDivotOffset);
    Pose2d blueStationDriverLeft = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.bluePickupLeft13,  -0*Constants.Measurements.coralStationDivotOffset);
    Pose2d redStationDriverRight = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redPickupRight2,  0*Constants.Measurements.coralStationDivotOffset);
    Pose2d redStationDriverLeft = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redPickupLeft1,  -0*Constants.Measurements.coralStationDivotOffset);
    
    Pose2d blueBranchA = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefCenter18, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchAOffset);
    Pose2d blueBranchB = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefCenter18, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchBOffset);
    Pose2d blueBranchC = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefRight17, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchCOffset);
    Pose2d blueBranchD = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefRight17, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchDOffset);
    Pose2d blueBranchE = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackRight22, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchEOffset);
    Pose2d blueBranchF = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackRight22, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchFOffset);
    Pose2d blueBranchG = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackCenter21, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchGOffset);
    Pose2d blueBranchH = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackCenter21, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchHOffset);
    Pose2d blueBranchI = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackLeft20, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchIOffset);
    Pose2d blueBranchJ = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackLeft20, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchJOffset);
    Pose2d blueBranchK = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefLeft19, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchKOffset);
    Pose2d blueBranchL = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefLeft19, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchLOffset);

    Reef blueReefCenter18 = new Reef(Constants.RobotPositions.blueReefCenter18, blueBranchA, blueBranchB);
    Reef blueReefRight17 = new Reef(Constants.RobotPositions.blueReefRight17, blueBranchC, blueBranchD);
    Reef blueReefBackRight22 = new Reef(Constants.RobotPositions.blueReefBackRight22, blueBranchE, blueBranchF);
    Reef blueReefBackCenter21 = new Reef(Constants.RobotPositions.blueReefBackCenter21, blueBranchG, blueBranchH);
    Reef blueReefBackLeft20 = new Reef(Constants.RobotPositions.blueReefBackLeft20, blueBranchI, blueBranchJ);
    Reef blueReefLeft19 = new Reef(Constants.RobotPositions.blueReefLeft19, blueBranchK, blueBranchL);
    
    Pose2d redBranchA = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefCenter7, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchAOffset);
    Pose2d redBranchB = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefCenter7, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchBOffset);
    Pose2d redBranchC = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefRight8, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchCOffset);
    Pose2d redBranchD = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefRight8, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchDOffset);
    Pose2d redBranchE = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackRight9, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchEOffset);
    Pose2d redBranchF = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackRight9, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchFOffset);
    Pose2d redBranchG = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackCenter10, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchGOffset);
    Pose2d redBranchH = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackCenter10, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchHOffset);
    Pose2d redBranchI = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackLeft11, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchIOffset);
    Pose2d redBranchJ = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackLeft11, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchJOffset);
    Pose2d redBranchK = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefLeft6, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchKOffset);
    Pose2d redBranchL = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefLeft6, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchLOffset);
    
    Reef redReefCenter7 = new Reef(Constants.RobotPositions.redReefCenter7, redBranchA, redBranchB);
    Reef redReefRight8 = new Reef(Constants.RobotPositions.redReefRight8, redBranchC, redBranchD);
    Reef redReefBackRight9 = new Reef(Constants.RobotPositions.redReefBackRight9, redBranchE, redBranchF);
    Reef redReefBackCenter10 = new Reef(Constants.RobotPositions.redReefBackCenter10, redBranchG, redBranchH);
    Reef redReefBackLeft11 = new Reef(Constants.RobotPositions.redReefBackLeft11, redBranchI, redBranchJ);
    Reef redReefLeft6 = new Reef(Constants.RobotPositions.redReefLeft6, redBranchK, redBranchL);

    PathPlannerPath B_to_station, C_to_station, D_to_station, E_to_station, K_to_station, L_to_station,
    station_to_B, station_to_C, station_to_D, station_to_A, station_to_K, station_to_L, G_to_driver_right, G_to_driver_left,
    simple_auto;
    
    public enum AUTO {
        BLUE_DRIVER_LEFT, BLUE_CENTER, BLUE_DRIVER_RIGHT,
        RED_DRIVER_LEFT, RED_CENTER, RED_DRIVER_RIGHT,
        PRACTICE_FIELD, SIMPLE_AUTO
    }
    private SendableChooser<AUTO> chooser;
    private SendableChooser<Command> ppChooser;
    StructPublisher<Pose2d> reefPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Auto Reef Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> stationPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Auto Station Pose", Pose2d.struct).publish();
    
    public Autos(){

    try {
        B_to_station = PathPlannerPath.fromChoreoTrajectory("B_to_station");
        C_to_station = PathPlannerPath.fromChoreoTrajectory("C_to_station");
        D_to_station = PathPlannerPath.fromChoreoTrajectory("D_to_station");
        E_to_station = PathPlannerPath.fromChoreoTrajectory("E_to_station");
        K_to_station = PathPlannerPath.fromChoreoTrajectory("K_to_station");
        L_to_station = PathPlannerPath.fromChoreoTrajectory("L_to_station");
        station_to_B = PathPlannerPath.fromChoreoTrajectory("station_to_B");
        station_to_C = PathPlannerPath.fromChoreoTrajectory("station_to_C");
        station_to_D = PathPlannerPath.fromChoreoTrajectory("station_to_D");
        station_to_A = PathPlannerPath.fromChoreoTrajectory("station_to_A");
        station_to_K = PathPlannerPath.fromChoreoTrajectory("station_to_K");
        station_to_L = PathPlannerPath.fromChoreoTrajectory("station_to_L");
        G_to_driver_right = PathPlannerPath.fromChoreoTrajectory("G_to_driver_right");
        G_to_driver_left = PathPlannerPath.fromChoreoTrajectory("G_to_driver_left");
        simple_auto = PathPlannerPath.fromChoreoTrajectory("simple_auto");
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }

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
        chooser.addOption("simple", AUTO.SIMPLE_AUTO);
        chooser.setDefaultOption("Default", null);
        
        
        ppChooser = AutoBuilder.buildAutoChooser();
        // // Put the auto chooser on the dashboard
        SmartDashboard.putData("AUTOS", chooser);
        SmartDashboard.putData("PP AUTOS", ppChooser);
        
        // 1. Create trajectory settings
        this.trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        thetaController.enableContinuousInput(0, 360);
    }

    public Command getAuto() {
        // return ppChooser.getSelected();
        AUTO auto = chooser.getSelected();
        if (auto == null) {
            System.out.println("auto is null");
            return new InstantCommand();
        }

        return switch (auto) {
            case BLUE_DRIVER_LEFT -> getBlueDriverLeft();
            case BLUE_CENTER -> getBlueCenter();
            case BLUE_DRIVER_RIGHT -> getBlueDriverRight();
            case RED_DRIVER_LEFT -> getRedDriverLeft();
            case RED_CENTER -> getRedCenter();
            case RED_DRIVER_RIGHT -> getRedDriverRight();
            case PRACTICE_FIELD -> getPracticeField();
            case SIMPLE_AUTO -> getSimpleAuto();
            default -> new InstantCommand();
        };
    }

    public Command getSimpleAuto() {
        return getStationCommmand(simple_auto);
    }

    public Command getBlueDriverLeft() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), blueBranchI, AutoConstants.firstWait)
        );
    }

    public Command getPracticeField() {
        return getScoreCommand(swerveSubsystem.getPose(), redBranchK, AutoConstants.firstWait);
    }

    public Command getBlueCenter() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), blueBranchG, AutoConstants.firstWait)
        );
    }

    public Command getBlueDriverRight() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), blueBranchE, AutoConstants.firstWait),
            getStationCommmand(swerveSubsystem.offsetPoint(blueStationDriverRight, 0.05, 0.0, 0), AutoConstants.stationWait),
            getScoreCommand(blueBranchC, AutoConstants.secondWait),
            getStationCommmand(swerveSubsystem.offsetPoint(blueStationDriverRight, 0.01, -0.03, 0), AutoConstants.stationWait),
            getScoreCommand(blueBranchD, AutoConstants.secondWait),
            getStationCommmand(blueStationDriverRight, AutoConstants.stationWait)
        );
    }

    public Command getRedDriverLeft() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), redBranchI, AutoConstants.firstWait),
            getStationCommand(redStationDriverLeft, redStationDriverLeft),
            getScoreCommand(redBranchK, AutoConstants.stationWait),
            getStationCommand(redStationDriverLeft, redStationDriverLeft),
            getScoreCommand(redBranchL, AutoConstants.stationWait)
        );
    }

    public Command getRedCenter() {
        return getScoreCommand(swerveSubsystem.getPose(), redBranchG, AutoConstants.firstWait);
    }

    public Command getRedDriverRight() {
        return new SequentialCommandGroup(
            getScoreCommand(swerveSubsystem.getPose(), redBranchE, AutoConstants.firstWait),
            getStationCommmand(swerveSubsystem.offsetPoint(redStationDriverRight, /* for practice */ 0.05, 0.0, 0), AutoConstants.stationWait),
            getScoreCommand(redBranchC, AutoConstants.secondWait),
            getStationCommmand(swerveSubsystem.offsetPoint(redStationDriverRight, 0.01, -0.03, 0), AutoConstants.stationWait),
            getScoreCommand(redBranchD, AutoConstants.secondWait),
            getStationCommmand(redStationDriverRight, AutoConstants.stationWait)
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

    // edu.wpi.first.math.trajectory.Trajectory traj = TrajectoryGenerator.generateTrajectory(
    //   startPose,
    //   List.of(),
    //   endPose,
    //   trajectoryConfig);

    // // 4. Construct command to follow trajectory
    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //   traj,
    //   swerveSubsystem::getPose,
    //   DriveConstants.kDriveKinematics,
    //   xController,
    //   yController,
    //   thetaController,
    //   swerveSubsystem::setModuleStates,
    //   swerveSubsystem);

    Command swerveControllerCommand = AutoBuilder.pathfindToPose(endPose, new PathConstraints(3.0, 1.3, 540, 720));

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
                    new InstantCommand(() -> RobotContainer.coralIntakeSubsystem.runIntake(1))
                )
            )
        ),
        new WaitCommand(0.4), //TODO: Change wait time
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
  public Command getScoreCommand(Pose2d endPose, double waitTime) {
    // An example command will be run in autonomous

    if (endPose == null) {
      return new InstantCommand();
    }
    reefPosePublisher.set(endPose);

    Command swerveControllerCommand = AutoBuilder.pathfindToPose(endPose, new PathConstraints(3.0, 1.3, 540, 720));

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
  public Command getScoreCommand(PathPlannerPath path, double waitTime) {
    // An example command will be run in autonomous
    Command followChoreoPath;
    try {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        followChoreoPath = AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        followChoreoPath = Commands.none();
    }
    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        // Drive then move elevator after 1 second
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                followChoreoPath,
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

    edu.wpi.first.math.trajectory.Trajectory traj = TrajectoryGenerator.generateTrajectory(
      swerveSubsystem.offsetPoint(startPose, 0, -2, 0),
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getStationCommmand(PathPlannerPath path) {
    Command followChoreoPath;
    try {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        followChoreoPath = AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        followChoreoPath = Commands.none();
    }

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
                    followChoreoPath
                )
            )
        )
    );
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getStationCommmand(Pose2d endPose, double waitTime) {
    Command swerveControllerCommand = AutoBuilder.pathfindToPose(swerveSubsystem.offsetPoint(endPose, 0, 0, 0), new PathConstraints(3.0, 1.3, 540, 720));
    // Command swerveControllerCommand2 = AutoBuilder.pathfindToPose(swerveSubsystem.offsetPoint(endPose, 0, 0.3, 0), new PathConstraints(1.0, 1.0, 540, 720));

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        // Move elevator then drive after 1.5 seconds
        new ParallelCommandGroup(
            new ParallelCommandGroup(
                new MoveElevator(RobotContainer.elevatorSubsystem, ElevatorStates.L1),
                new MoveArm(RobotContainer.coralArmSubsystem, CoralArmStates.PICKUP)
            ),
            new SequentialCommandGroup(
                new WaitCommand(waitTime),

                new ParallelCommandGroup(
                    swerveControllerCommand,
                    new Intake(RobotContainer.coralIntakeSubsystem)
                )
            )
        )
    );
  }
}
