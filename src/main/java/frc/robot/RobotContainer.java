// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.DeepClimb;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoralArmStates;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.RobotPositions;
import frc.robot.Constants.USB;
import frc.robot.commands.autonomous.Autos;
import frc.robot.commands.autonomous.MoveArm;
import frc.robot.commands.autonomous.MoveElevator;
import frc.robot.commands.autonomous.Shoot;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final static Joystick driverController = new Joystick(USB.DRIVER_CONTROLLER);
  public final static Joystick operatorController = new Joystick(USB.OPERATOR_CONTROLLER);
  public final static Elevator elevatorSubsystem = new Elevator();
  public final static CoralArm coralArmSubsystem = new CoralArm();
  public final static CoralIntake coralIntakeSubsystem = new CoralIntake();
  public final static DeepClimb deepClimbSubsystem = new DeepClimb();
  public final static LEDs led = new LEDs();
  public final static RobotState robotState = new RobotState(swerveSubsystem, elevatorSubsystem, coralArmSubsystem, coralIntakeSubsystem, deepClimbSubsystem);
  // 1. Create trajectory settings
  TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

  private boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red); 

  public static final Autos auto = new Autos();
  // 3. Define PID controllers for tracking trajectory
  PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  ProfiledPIDController thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  public static double wantedAngle = -1;
  public static int shouldAutoFixDrift = 1; // 1 = auto drift, 2 = auto align, 0 = none
  public static int gameState = GameConstants.Robot;
  public static Trajectory currentTrajectory = null;
  public static Pose2d goalPose = null;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    thetaController.enableContinuousInput(0, 360);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getScoreCommand(Pose2d endPose) {
    // An example command will be run in autonomous

    if (endPose == null) {
      return new InstantCommand();
    }
    
    Trajectory traj = TrajectoryGenerator.generateTrajectory(
      RobotContainer.swerveSubsystem.poseEstimator.getEstimatedPosition(),
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
      // new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules()),
      new ParallelCommandGroup(
        new MoveElevator(elevatorSubsystem, ElevatorStates.L4),
        new MoveArm(coralArmSubsystem, CoralArmStates.L4)
      ),
      new InstantCommand(() -> coralIntakeSubsystem.runIntake(-1))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getStationAuto(Trajectory trajectory) {
    // 4. Construct command to follow trajectory 
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
      new WaitCommand(1.0),
      new ParallelCommandGroup(
        new MoveElevator(elevatorSubsystem, ElevatorStates.L1),
        new MoveArm(coralArmSubsystem, CoralArmStates.PICKUP)
      ),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules()),
      new InstantCommand(() -> coralIntakeSubsystem.runIntake(1))
    );
  }
}
