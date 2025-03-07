// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.TeleCommandGroup;
// import frc.robot.commands.autonomous.Autos;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.Colors;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  // Loads a swerve trajectory, alternatively use DifferentialSample if the robot is tank drive
  private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("Simple Auto");

  private final Timer timer = new Timer();

  private Command m_autonomousCommand;

  private RobotContainer robotContainer;
  
  private AutoChooser autoChooser;

  private double autoStartTimer = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // // Create the auto chooser
    // autoChooser = new AutoChooser();

    // // Add options to the chooser
    // autoChooser.addCmd("simple auto", RobotContainer.auto::simpleAuto);
    // // autoChooser.addRoutine("auto2", RobotContainer.auto::auto2);
    // autoChooser.addRoutine("NONE", RobotContainer.auto::noneAuto);

    // // Put the auto chooser on the dashboard
    // SmartDashboard.putData("AUTOS", autoChooser);

    // Schedule the selected auto during the autonomous period
    // RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());


    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, Constants.LimelightConstants.tagName + ".local", port);
      PortForwarder.add(port+10, Constants.LimelightConstants.gamePieceName + ".local", port);
      PortForwarder.add(port+20, Constants.LimelightConstants.driverName + ".local", port);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.gameState = GameConstants.Robot;
    RobotContainer.elevatorSubsystem.setState(Constants.ElevatorStates.NONE);
    RobotContainer.coralArmSubsystem.setState(Constants.CoralArmStates.NONE);
    RobotContainer.coralIntakeSubsystem.setState(Constants.CoralIntakeStates.NONE);
    RobotContainer.robotState.setState(Constants.RobotStates.NONE);

  }

  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStartTimer = Timer.getTimestamp();
    
    // if (trajectory.isPresent()) {
    //     // Get the initial pose of the trajectory
    //     Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

    //     if (initialPose.isPresent()) {
    //         // Reset odometry to the start of the trajectory

    //     }
    // }
    RobotContainer.swerveSubsystem.zeroHeading();
    // RobotContainer.swerveSubsystem.resetOdometry(RobotContainer.swerveSubsystem.poseEstimator.getEstimatedPosition());
    //     }
    // }

    // Reset and start the timer when the autonomous period begins
    timer.restart();
    
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule(); // Today
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    RobotContainer.gameState = GameConstants.Auto;
    ChassisSpeeds autoChassisSpeeds = new ChassisSpeeds(0, 0.5, 0);
    ChassisSpeeds stopChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] autoState = DriveConstants.kDriveKinematics.toSwerveModuleStates(autoChassisSpeeds);
    SwerveModuleState[] stopState = DriveConstants.kDriveKinematics.toSwerveModuleStates(stopChassisSpeeds);
      
      // if (RobotContainer.gameState == GameConstants.Auto){
      //   if (Math.abs(autoStartTimer - Timer.getTimestamp()) < 7){
      //     RobotContainer.swerveSubsystem.setModuleStates(autoState);
      //   }
      //   else {
      //     RobotContainer.swerveSubsystem.setModuleStates(stopState);
      //   }
      // }
      
    // if (trajectory.isPresent()) {
    //   // 
    //   Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

    //   if (sample.isPresent()) {
    //     RobotContainer.swerveSubsystem.followTrajectory(sample.get());
    //   }
    // }
  }

  @Override
  public void teleopInit() {
    RobotContainer.gameState = GameConstants.TeleOp;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    RobotContainer.swerveSubsystem.setDefaultCommand(
      new TeleCommandGroup(
        RobotContainer.robotState,
        RobotContainer.swerveSubsystem,
        RobotContainer.driverController,
        RobotContainer.operatorController,
        RobotContainer.elevatorSubsystem,
        RobotContainer.coralArmSubsystem,
        RobotContainer.coralIntakeSubsystem,
        RobotContainer.deepClimbSubsystem
      )
    );
    RobotContainer.elevatorSubsystem.setState(Constants.ElevatorStates.NONE);
    RobotContainer.coralArmSubsystem.setState(Constants.CoralArmStates.NONE);
    RobotContainer.coralIntakeSubsystem.setState(Constants.CoralIntakeStates.NONE);
    RobotContainer.robotState.setState(Constants.RobotStates.NONE);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
  
  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);  
  }
}
