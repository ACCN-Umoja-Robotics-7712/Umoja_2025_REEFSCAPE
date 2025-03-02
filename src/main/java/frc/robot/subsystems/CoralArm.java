package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
// Intake for the 2025 robot
public class CoralArm extends SubsystemBase {

    SparkMax coralArmMotor;
    RelativeEncoder coralArmEncoder;  
    private double state = Constants.CoralArmStates.PICKUP;
    PIDController coralArmPID = new PIDController(Constants.CoralConstants.kP, 0, 0);

    public CoralArm(){

        SparkMaxConfig coralArmConfig = new SparkMaxConfig();
        coralArmConfig.idleMode(IdleMode.kBrake);
        coralArmConfig.inverted(true);

        coralArmMotor = new SparkMax(Constants.CoralConstants.coralArmMotorID, MotorType.kBrushless); 
        coralArmConfig.softLimit.forwardSoftLimitEnabled(true); //TODO: Change to true once you get limits fixed
        coralArmConfig.softLimit.reverseSoftLimitEnabled(true);
        coralArmConfig.softLimit.forwardSoftLimit(Constants.CoralConstants.coralArmTopLimit);
        coralArmConfig.softLimit.reverseSoftLimit(Constants.CoralConstants.coralArmBottomLimit);
        coralArmMotor.configure(coralArmConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        coralArmEncoder = coralArmMotor.getEncoder();
        
    }
    
    public void runArm(double percent){
        if(RobotContainer.elevatorSubsystem.getEncoder() < Constants.ElevatorConstants.elevatorArmLimit && percent > 0 && coralArmEncoder.getPosition() > Constants.CoralConstants.coralArmElevatorLimit){
            coralArmMotor.set(0);
        }
        else {
            coralArmMotor.set(percent); // TODO: Add thing to not let it hit into robot if elevator is under a certain thing 
        }
    }

    public boolean isClimbReady(){
        return coralArmEncoder.getPosition() > Constants.CoralArmStates.CLIMB; // TODO: subtract number for leeway
    }

    public void setState(double coralArmState) {
        state = coralArmState;
    }

    public double getEncoder(){
        return coralArmEncoder.getPosition();
    }
    public double getState(){
        return state;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Intake Pos", intakeEncoder.getPosition());
        SmartDashboard.putNumber("Coral Arm Position", coralArmEncoder.getPosition());
        SmartDashboard.putNumber("Coral Arm State", state);
        if(state != Constants.CoralArmStates.NONE){
            // runArm(coralArmPID.calculate(coralArmEncoder.getPosition(), state));
        }
    }
}


// Let the rookies try to finish the code