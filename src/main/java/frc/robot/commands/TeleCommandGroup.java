package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.XBoxConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DeepClimb;

public class TeleCommandGroup extends ParallelCommandGroup{
    public TeleCommandGroup(RobotState robotState, SwerveSubsystem swerveSubsystem, Joystick driverController, Joystick operatorController, Elevator elevatorSubsystem, CoralArm coralArmSubsystem, CoralIntake coralIntakeSubsystem, DeepClimb deepClimbSubsystem){
        
        boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
        int flip = isBlue ? -1 : 1;
        //Driver Command
        addCommands(
            new SwerveJoystick(
                swerveSubsystem,
                () -> flip*driverController.getRawAxis(XBoxConstants.LY),
                () -> flip*driverController.getRawAxis(XBoxConstants.LX),
                () -> -driverController.getRawAxis(XBoxConstants.RX)
            )
        );

                // operator
        addCommands(
            new OperatorJoystick(robotState, elevatorSubsystem, coralArmSubsystem, coralIntakeSubsystem, deepClimbSubsystem, operatorController)
                );
    }
}

