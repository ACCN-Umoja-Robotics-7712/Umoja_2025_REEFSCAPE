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
public class AlgaeArm extends SubsystemBase {

    SparkMax algaeArmMotor;
    RelativeEncoder algaeArmEncoder;
    
    public AlgaeArm(){

        SparkMaxConfig algaeArmConfig = new SparkMaxConfig();
        algaeArmConfig.idleMode(IdleMode.kBrake);
        algaeArmConfig.inverted(false);

        algaeArmMotor = new SparkMax(Constants.AlgaeConstants.algaeIntakeMotorID, MotorType.kBrushless); 
        algaeArmMotor.configure(algaeArmConfig, null, null);
        
        algaeArmEncoder = algaeArmMotor.getEncoder();
        
    }
    
    public void runIntake(double percent){
        algaeArmMotor.set(percent);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Pos", algaeArmEncoder.getPosition());
    }
}

// Let the rookies try to finish the code