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
public class CoralArm extends SubsystemBase {

    SparkMax coralArmMotor;
    RelativeEncoder coralArmEncoder;  
    
    public CoralArm(){

        SparkMaxConfig coralArmConfig = new SparkMaxConfig();
        coralArmConfig.idleMode(IdleMode.kBrake);
        coralArmConfig.inverted(false);

        coralArmMotor = new SparkMax(Constants.CoralConstants.coralArmMotorID, MotorType.kBrushless); 
        coralArmMotor.configure(coralArmConfig, null, null);
        
        coralArmEncoder = coralArmMotor.getEncoder();
        
    }
    
    public void runIntake(double percent){
        coralArmMotor.set(percent);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Intake Pos", intakeEncoder.getPosition());
    }
}


// Let the rookies try to finish the code