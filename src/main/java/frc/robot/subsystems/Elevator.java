package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Colors;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Constants.GameConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor1 = new SparkMax(Constants.ElevatorConstants.elevatorMotor1ID, MotorType.kBrushless);
    private final SparkMax elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.elevatorMotor2ID, MotorType.kBrushless);

    private final RelativeEncoder elevator1Encoder = elevatorMotor1.getEncoder();
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(3);
    ProfiledPIDController elevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, 0, ElevatorConstants.kElevatorConstraints);
    // PIDController elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, 0);

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

        elevatorPID.setTolerance(1);

    }

    public void runElevator(double percent){
        if (isDangerous(percent)){
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
        return elevator1Encoder.getPosition() >= Constants.ElevatorStates.CLIMB; // TODO: Subtract number for leeway
    }

    public void setState(double state) {
        if (this.state != state) {
            elevatorPID.reset(elevator1Encoder.getPosition());
            this.state = state;
        }
    }

    public double getState(){
        return state;
    }

    public boolean didReachState() {
        return elevatorPID.atGoal();
    }

    public boolean isDangerous(double percent) {
        boolean isArmBlocking = RobotContainer.coralArmSubsystem.getEncoder() > Constants.CoralConstants.coralArmElevatorLimit;
        boolean isMovingDown = percent < 0;
        boolean isCloseToArm = elevator1Encoder.getPosition() < Constants.ElevatorConstants.elevatorArmLimit;
        return  isArmBlocking && isMovingDown && isCloseToArm;
    }

    public boolean isDangerousState(double state) {
        boolean isArmBlocking = RobotContainer.coralArmSubsystem.getEncoder() > Constants.CoralConstants.coralArmElevatorLimit;
        boolean isMovingDown = state < elevator1Encoder.getPosition();
        return  isArmBlocking && isMovingDown;
    }

    public void stop(){
        state = ElevatorStates.NONE;
        elevatorMotor1.set(0);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        if (state != ElevatorStates.NONE) {
            if (!isDangerousState(state)) {
                runElevator(elevatorPID.calculate(elevator1Encoder.getPosition(), state));
            } else {
                runElevator(0);
            }
        }
        SmartDashboard.putNumber("Elevator State", state);
        SmartDashboard.putNumber("ELEVATOR ENCODER", elevator1Encoder.getPosition());
        SmartDashboard.putNumber("ELEVATOR ABSOLUTE ENCODER", absoluteEncoder.get());
        // SmartDashboard.putNumber("Elevator Velocity", elevatorMotor1;
    }
}
