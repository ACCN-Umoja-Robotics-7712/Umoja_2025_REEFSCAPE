package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// Intake for the 2025 robot
public class AlgaeIntake extends SubsystemBase {

    SparkMax algaeIntakeMotor;
    RelativeEncoder algaeIntakeEncoder;
    
    public AlgaeIntake(){

        SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
        algaeIntakeConfig.idleMode(IdleMode.kBrake);
        algaeIntakeConfig.inverted(false);

        algaeIntakeMotor = new SparkMax(Constants.AlgaeConstants.algaeIntakeMotorID, MotorType.kBrushless); 
        algaeIntakeMotor.configure(algaeIntakeConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        algaeIntakeEncoder = algaeIntakeMotor.getEncoder();
        
    }
    
    public void runIntake(double percent){
        algaeIntakeMotor.set(percent);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Pos", algaeIntakeEncoder.getPosition());
    }
}

// Let the rookies try to finish the code