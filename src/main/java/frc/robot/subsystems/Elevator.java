package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorStates;
import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor1 = new SparkMax(Constants.ElevatorConstants.elevatorMotor1ID, MotorType.kBrushless);
    private final SparkMax elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.elevatorMotor2ID, MotorType.kBrushless);

    private final RelativeEncoder elevator1Encoder = elevatorMotor1.getEncoder();
    PIDController elevatorPID = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, 0);

    private double state = Constants.ElevatorStates.NONE;
    
    public Elevator() {
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        SparkMaxConfig elevator2Config = new SparkMaxConfig();

        
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.inverted(false);
        elevatorConfig.softLimit.forwardSoftLimitEnabled(true);
        elevatorConfig.softLimit.reverseSoftLimitEnabled(true);
        elevatorConfig.softLimit.forwardSoftLimit(Constants.ElevatorConstants.elevatorTopLimit);
        elevatorConfig.softLimit.reverseSoftLimit(Constants.ElevatorConstants.elevatorBottomLimit);
        elevatorMotor1.configure(elevatorConfig,ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);

        elevator2Config.idleMode(IdleMode.kBrake);
        elevator2Config.follow(elevatorMotor1.getDeviceId(), true);
        elevatorMotor2.configure(elevator2Config, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);

    }

    public void runElevator(double percent){
        if (RobotContainer.coralArmSubsystem.getEncoder() > Constants.CoralConstants.coralArmElevatorLimit && percent < 0 && elevator1Encoder.getPosition() < Constants.ElevatorConstants.elevatorArmLimit){
            elevatorMotor1.set(0);
        }
        else {
            elevatorMotor1.set(percent);
        }
    }
    public double getEncoder(){
        return elevator1Encoder.getPosition();
    }

    public boolean isClimbReady(){
        return elevator1Encoder.getPosition() >= Constants.ElevatorStates.CLIMB; // To-do: Subtract number for leeway
    }

    public void setState(double state) {
        this.state = state;
    }

    public double getState(){
        return state;
    }

    public void stop(){
        elevatorMotor1.set(0);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        // if (state != ElevatorStates.NONE) {
        //     runElevator(elevatorPID.calculate(elevator1Encoder.getPosition(), state));
        // }
        SmartDashboard.putNumber("Elevator State", state);
        SmartDashboard.putNumber("ELEVATOR ENCODER", elevator1Encoder.getPosition());
    }
}
