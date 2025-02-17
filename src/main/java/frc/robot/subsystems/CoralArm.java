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
// Intake for the 2025 robot
public class CoralArm extends SubsystemBase {

    SparkMax coralArmMotor;
    RelativeEncoder coralArmEncoder;  
    double state = Constants.CoralArmStates.pickup;
    PIDController coralArmPID = new PIDController(Constants.CoralConstants.kP, 0, 0);

    public CoralArm(){

        SparkMaxConfig coralArmConfig = new SparkMaxConfig();
        coralArmConfig.idleMode(IdleMode.kBrake);
        coralArmConfig.inverted(false);

        coralArmMotor = new SparkMax(Constants.CoralConstants.coralArmMotorID, MotorType.kBrushless); 
        coralArmMotor.configure(coralArmConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        coralArmEncoder = coralArmMotor.getEncoder();
        
    }
    
    public void runArm(double percent){
        coralArmMotor.set(percent);
    }
    public void setState(double coralArmState) {
        state = coralArmState;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Intake Pos", intakeEncoder.getPosition());
        if(state != -1){
            runArm(coralArmPID.calculate(coralArmEncoder.getPosition(), state));
        }
    }
}


// Let the rookies try to finish the code