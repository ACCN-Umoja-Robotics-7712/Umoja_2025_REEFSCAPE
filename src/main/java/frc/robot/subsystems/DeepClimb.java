package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DeepClimbStates;
import frc.robot.Constants.DeepClimbConstants;;

// Intake for the 2025 robot
public class DeepClimb extends SubsystemBase {

    SparkMax deepClimbMotor;
    RelativeEncoder deepClimbEncoder;  
    private double state = Constants.DeepClimbStates.NONE; 
    PIDController deepClimbPID = new PIDController(DeepClimbConstants.kP, DeepClimbConstants.kI, 0);

    
    public DeepClimb(){

        SparkMaxConfig deepClimbConfig = new SparkMaxConfig();
        deepClimbConfig.idleMode(IdleMode.kBrake);
        deepClimbConfig.inverted(false);

        deepClimbMotor = new SparkMax(Constants.DeepClimbConstants.climbMotorID, MotorType.kBrushless);
        deepClimbMotor.configure(deepClimbConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        deepClimbEncoder = deepClimbMotor.getEncoder();
        
    }
    
    public void runClimber(double percent){
        deepClimbMotor.set(percent);
    }

    public double getState(){
        return state;
    }

    public void setState(double state){
        this.state = state; 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber current: ", deepClimbMotor.getOutputCurrent());

        if (state != DeepClimbStates.NONE) {
            runClimber(deepClimbPID.calculate(deepClimbEncoder.getPosition(), state));
        }
    }
}


// Let the rookies try to finish the code