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
import frc.robot.Constants;
import frc.robot.Constants.CoralIntakeStates;
// Intake for the 2025 robot
public class CoralIntake extends SubsystemBase {

    SparkMax coralIntakeMotor;
    RelativeEncoder coralIntakeEncoder;  
    private double state = Constants.CoralIntakeStates.NONE;
    
    public CoralIntake(){

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.inverted(false);

        coralIntakeMotor = new SparkMax(Constants.CoralConstants.coralIntakeMotorID, MotorType.kBrushless); 
        coralIntakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        coralIntakeEncoder = coralIntakeMotor.getEncoder();
        
    }
    
    public void runIntake(double percent){
        coralIntakeMotor.set(percent);
    }

    public boolean hasCoral(){
        double diff = coralIntakeMotor.getOutputCurrent() - Constants.CoralConstants.coralCurrent;
        if (diff < 3){
            return true;
        } 
        else{
            return false;
        }
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
        SmartDashboard.putNumber("Coral current: ", coralIntakeMotor.getOutputCurrent());

        if (state == Constants.CoralIntakeStates.INTAKE) {
            runIntake(0.05);
        } else if (state == Constants.CoralIntakeStates.READY || state == Constants.CoralIntakeStates.NONE) {
            runIntake(0);
        } else if (state == Constants.CoralIntakeStates.SHOOTING) {
            runIntake(-0.1);
        }
    }
}


// Let the rookies try to finish the code