package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// Intake for the 2025 robot
public class CoralIntake extends SubsystemBase {

    SparkMax intakeMotor;
    RelativeEncoder intakeEncoder;  
    
    public CoralIntake(){

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.inverted(false);

        intakeMotor = new SparkMax(Constants.CoralConstants.CoralMotorId, MotorType.kBrushless); 
        intakeMotor.configure(intakeConfig, null, null);
        
        intakeEncoder = intakeMotor.getEncoder();
        
    }
    
    public void runIntake(double percent){
        intakeMotor.set(percent);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Pos", intakeEncoder.getPosition());
    }
}


// Let the rookies try to finish the code