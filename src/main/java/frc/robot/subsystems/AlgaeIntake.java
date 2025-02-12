package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// Intake for the 2025 robot
public class AlgaeIntake extends SubsystemBase {

    SparkMax AlgaeIntakeMotor;
    RelativeEncoder AlgaeIntakeEncoder;
    
    public AlgaeIntake(){

        SparkMaxConfig AlgaeIntakeConfig = new SparkMaxConfig();
        AlgaeIntakeConfig.idleMode(IdleMode.kBrake);
        AlgaeIntakeConfig.inverted(false);

        AlgaeIntakeMotor = new SparkMax(Constants.CoralConstants.CoralMotorId, MotorType.kBrushless); 
        AlgaeIntakeMotor.configure(AlgaeIntakeConfig, null, null);
        
        AlgaeIntakeEncoder = AlgaeIntakeMotor.getEncoder();
        
    }
    
    public void runIntake(double percent){
        AlgaeIntakeMotor.set(percent);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Pos", AlgaeIntakeEncoder.getPosition());
    }
}

// Let the rookies try to finish the code