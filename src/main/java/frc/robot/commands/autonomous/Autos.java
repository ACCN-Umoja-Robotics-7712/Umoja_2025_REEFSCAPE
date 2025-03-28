package frc.robot.commands.autonomous;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoralArmStates;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.objects.Point;
import frc.robot.subsystems.SwerveSubsystem;

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
    
    Pose2d blueBranchA = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefCenter18, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchAOffset.getFirst(), Constants.BranchOffsets.blueBranchAOffset.getSecond());
    Pose2d blueBranchB = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefCenter18, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchBOffset.getFirst(), Constants.BranchOffsets.blueBranchBOffset.getSecond());
    Pose2d blueBranchC = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefRight17, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchCOffset.getFirst(), Constants.BranchOffsets.blueBranchCOffset.getSecond());
    Pose2d blueBranchD = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefRight17, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchDOffset.getFirst(), Constants.BranchOffsets.blueBranchDOffset.getSecond());
    Pose2d blueBranchE = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackRight22, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchEOffset.getFirst(), Constants.BranchOffsets.blueBranchEOffset.getSecond());
    Pose2d blueBranchF = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackRight22, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchFOffset.getFirst(), Constants.BranchOffsets.blueBranchFOffset.getSecond());
    Pose2d blueBranchG = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackCenter21, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchGOffset.getFirst(), Constants.BranchOffsets.blueBranchGOffset.getSecond());
    Pose2d blueBranchH = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackCenter21, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchHOffset.getFirst(), Constants.BranchOffsets.blueBranchHOffset.getSecond());
    Pose2d blueBranchI = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackLeft20, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchIOffset.getFirst(), Constants.BranchOffsets.blueBranchIOffset.getSecond());
    Pose2d blueBranchJ = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefBackLeft20, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchJOffset.getFirst(), Constants.BranchOffsets.blueBranchJOffset.getSecond());
    Pose2d blueBranchK = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefLeft19, Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchKOffset.getFirst(), Constants.BranchOffsets.blueBranchKOffset.getSecond());
    Pose2d blueBranchL = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.blueReefLeft19, -Constants.Measurements.branchOffset + Constants.BranchOffsets.blueBranchLOffset.getFirst(), Constants.BranchOffsets.blueBranchLOffset.getSecond());

    public Point blueReefCenter18 = new Point(Constants.RobotPositions.blueReefCenter18, blueBranchA, blueBranchB);
    public Point blueReefRight17 = new Point(Constants.RobotPositions.blueReefRight17, blueBranchC, blueBranchD);
    public Point blueReefBackRight22 = new Point(Constants.RobotPositions.blueReefBackRight22, blueBranchE, blueBranchF);
    public Point blueReefBackCenter21 = new Point(Constants.RobotPositions.blueReefBackCenter21, blueBranchG, blueBranchH);
    public Point blueReefBackLeft20 = new Point(Constants.RobotPositions.blueReefBackLeft20, blueBranchI, blueBranchJ);
    public Point blueReefLeft19 = new Point(Constants.RobotPositions.blueReefLeft19, blueBranchK, blueBranchL);
    
    Pose2d redBranchA = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefCenter7, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchAOffset.getFirst(), Constants.BranchOffsets.blueBranchAOffset.getSecond());
    Pose2d redBranchB = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefCenter7, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchBOffset.getFirst(), Constants.BranchOffsets.blueBranchBOffset.getSecond());
    Pose2d redBranchC = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefRight8, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchCOffset.getFirst(), Constants.BranchOffsets.blueBranchCOffset.getSecond());
    Pose2d redBranchD = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefRight8, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchDOffset.getFirst(), Constants.BranchOffsets.blueBranchDOffset.getSecond());
    Pose2d redBranchE = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackRight9, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchEOffset.getFirst(), Constants.BranchOffsets.blueBranchEOffset.getSecond());
    Pose2d redBranchF = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackRight9, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchFOffset.getFirst(), Constants.BranchOffsets.blueBranchFOffset.getSecond());
    Pose2d redBranchG = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackCenter10, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchGOffset.getFirst(), Constants.BranchOffsets.blueBranchGOffset.getSecond());
    Pose2d redBranchH = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackCenter10, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchHOffset.getFirst(), Constants.BranchOffsets.blueBranchHOffset.getSecond());
    Pose2d redBranchI = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackLeft11, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchIOffset.getFirst(), Constants.BranchOffsets.blueBranchIOffset.getSecond());
    Pose2d redBranchJ = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefBackLeft11, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchJOffset.getFirst(), Constants.BranchOffsets.blueBranchJOffset.getSecond());
    Pose2d redBranchK = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefLeft6, Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchKOffset.getFirst(), Constants.BranchOffsets.blueBranchKOffset.getSecond());
    Pose2d redBranchL = RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redReefLeft6, -Constants.Measurements.branchOffset + Constants.BranchOffsets.redBranchLOffset.getFirst(), Constants.BranchOffsets.blueBranchLOffset.getSecond());
    
    public Point redReefCenter7 = new Point(Constants.RobotPositions.redReefCenter7, redBranchA, redBranchB);
    public Point redReefRight8 = new Point(Constants.RobotPositions.redReefRight8, redBranchC, redBranchD);
    public Point redReefBackRight9 = new Point(Constants.RobotPositions.redReefBackRight9, redBranchE, redBranchF);
    public Point redReefBackCenter10 = new Point(Constants.RobotPositions.redReefBackCenter10, redBranchG, redBranchH);
    public Point redReefBackLeft11 = new Point(Constants.RobotPositions.redReefBackLeft11, redBranchI, redBranchJ);
    public Point redReefLeft6 = new Point(Constants.RobotPositions.redReefLeft6, redBranchK, redBranchL);
    
    public List<Point> blueReef = Arrays.asList(blueReefRight17, blueReefCenter18, blueReefLeft19, blueReefBackLeft20, blueReefBackCenter21, blueReefBackRight22);
    public List<Point> redReef = Arrays.asList(redReefLeft6, redReefCenter7, redReefRight8, redReefBackRight9, redReefBackCenter10, redReefBackLeft11);
    
    public Point bluePickupLeft13 = new Point(Constants.RobotPositions.bluePickupLeft13, RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.bluePickupLeft13, 2*Constants.Measurements.coralStationDivotOffset), RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.bluePickupLeft13, -2*Constants.Measurements.coralStationDivotOffset));
    public Point bluePickupRight12 = new Point(Constants.RobotPositions.bluePickupRight12, RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.bluePickupRight12, 2*Constants.Measurements.coralStationDivotOffset), RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.bluePickupRight12, -2*Constants.Measurements.coralStationDivotOffset));
    public Point redPickupLeft1 = new Point(Constants.RobotPositions.redPickupLeft1, RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redPickupLeft1, 2*Constants.Measurements.coralStationDivotOffset), RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redPickupLeft1, -2*Constants.Measurements.coralStationDivotOffset));
    public Point redPickupRight2 = new Point(Constants.RobotPositions.redPickupRight2, RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redPickupRight2, 2*Constants.Measurements.coralStationDivotOffset), RobotContainer.swerveSubsystem.offsetPoint(Constants.RobotPositions.redPickupRight2, -2*Constants.Measurements.coralStationDivotOffset));
    
    public List<Point> blueStation = Arrays.asList(bluePickupLeft13, bluePickupRight12);
    public List<Point> redStation = Arrays.asList(redPickupLeft1, redPickupRight2);

    PathPlannerPath B_to_station, C_to_station, D_to_station, E_to_station, K_to_station, L_to_station,
    station_to_B, station_to_C, station_to_D, station_to_A, station_to_K, station_to_L, G_to_driver_right, G_to_driver_left,
    simple_auto;
    
    public enum AUTO {
        BLUE_DRIVER_LEFT, BLUE_CENTER, BLUE_DRIVER_RIGHT,
        RED_DRIVER_LEFT, RED_CENTER, RED_DRIVER_RIGHT,
        PRACTICE_FIELD, SIMPLE_AUTO, TUNE_AUTO
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
        chooser.addOption("tune", AUTO.TUNE_AUTO);
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
            case TUNE_AUTO -> getTuneAuto();
            default -> new InstantCommand();
        };
    }

    public Command getTuneAuto() {
        // drive forward/side 1 meter, turn 60 degrees
        return AutoBuilder.pathfindToPose(swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 1, 1, 60), Constants.pathConstraints);
    }

    public Command getSimpleAuto() {
        return getStationCommmand(simple_auto);
    }

    public Command getBlueDriverLeft() {
        return new SequentialCommandGroup(
            getScoreCommand(blueBranchI, AutoConstants.firstWait)
        );
    }

    public Command getPracticeField() {
        return getScoreCommand(redBranchK, AutoConstants.firstWait);
    }

    public Command getBlueCenter() {
        return new SequentialCommandGroup(
            getScoreCommand(blueBranchG, AutoConstants.firstWait),
            getAlgaeCommmand(blueBranchF, AutoConstants.firstWait)
        );
    }

    public Command getBlueDriverRight() {
        return new SequentialCommandGroup(
            getScoreCommand(blueBranchE, AutoConstants.firstWait),
            getStationCommmand(swerveSubsystem.offsetPoint(blueStationDriverRight, 0.05, 0.0), AutoConstants.stationWait),
            getScoreCommand(blueBranchC, AutoConstants.secondWait),
            getStationCommmand(swerveSubsystem.offsetPoint(blueStationDriverRight, 0.01, -0.03), AutoConstants.stationWait),
            getScoreCommand(blueBranchA, AutoConstants.secondWait),
            getStationCommmand(swerveSubsystem.offsetPoint(blueStationDriverRight, 0, -0.1), AutoConstants.stationWait)
        );
    }

    public Command getRedDriverLeft() {
        return new SequentialCommandGroup(
            getScoreCommand(redBranchI, AutoConstants.firstWait),
            getStationCommand(redStationDriverLeft, redStationDriverLeft),
            getScoreCommand(redBranchK, AutoConstants.stationWait),
            getStationCommand(redStationDriverLeft, redStationDriverLeft),
            getScoreCommand(redBranchL, AutoConstants.stationWait)
        );
    }

    public Command getRedCenter() {
        return getScoreCommand(redBranchG, AutoConstants.firstWait);
    }

    public Command getRedDriverRight() {
        return new SequentialCommandGroup(
            getScoreCommand(redBranchE, AutoConstants.firstWait),
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
  public Command getScoreCommand(Pose2d endPose, double waitTime) {
    // An example command will be run in autonomous

    if (endPose == null) {
      return new InstantCommand();
    }
    reefPosePublisher.set(endPose);

    Command swerveControllerCommand = AutoBuilder.pathfindToPose(endPose, Constants.pathConstraints);

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
        new WaitCommand(0.4), //TODO: Change wait time
        new Shoot(RobotContainer.coralIntakeSubsystem)
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
                    new Intake(RobotContainer.coralIntakeSubsystem).andThen(new Intake(RobotContainer.coralIntakeSubsystem)),
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
    Command swerveControllerCommand = AutoBuilder.pathfindToPose(swerveSubsystem.offsetPoint(endPose, 0, 0, 0), Constants.pathConstraints, 0.5);
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
  public Command getAlgaeCommmand(Pose2d endPose, double waitTime) {
    Command swerveControllerCommand = AutoBuilder.pathfindToPose(swerveSubsystem.offsetPoint(endPose, 0, 0, 0), Constants.pathConstraints, 0.5);
    // Command swerveControllerCommand2 = AutoBuilder.pathfindToPose(swerveSubsystem.offsetPoint(endPose, 0, 0.3, 0), new PathConstraints(1.0, 1.0, 540, 720));

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        // Move elevator then drive after 1.5 seconds
        new ParallelCommandGroup(
            
            new ParallelCommandGroup(
                new MoveElevator(RobotContainer.elevatorSubsystem, ElevatorStates.L1),
                new MoveArm(RobotContainer.coralArmSubsystem, CoralArmStates.L23)
            )
        ),
        swerveControllerCommand,
        new ParallelCommandGroup(
        new MoveElevator(RobotContainer.elevatorSubsystem, ElevatorStates.ALGAE),
        new TimedShoot(RobotContainer.coralIntakeSubsystem)
        )
    );
  }
}
