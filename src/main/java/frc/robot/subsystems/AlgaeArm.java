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
import edu.wpi.first.math.controller.PIDController;


// Intake for the 2025 robot
public class AlgaeArm extends SubsystemBase {

    SparkMax algaeArmMotor;
    RelativeEncoder algaeArmEncoder;
    PIDController algaeArmPID = new PIDController(Constants.AlgaeConstants.kP, 0,0);
    private double state = Constants.AlgaeModeStates.NONE;
    
    public AlgaeArm(){

        SparkMaxConfig algaeArmConfig = new SparkMaxConfig();
        algaeArmConfig.idleMode(IdleMode.kBrake);
        algaeArmConfig.inverted(false);

        algaeArmMotor = new SparkMax(Constants.AlgaeConstants.algaeIntakeMotorID, MotorType.kBrushless); 
        algaeArmMotor.configure(algaeArmConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        algaeArmEncoder = algaeArmMotor.getEncoder();
        
    }
    
    public void runAlgaeArm(double percent){
        algaeArmMotor.set(percent*Constants.AlgaeConstants.armLimiter);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Pos", algaeArmEncoder.getPosition());

        if (state != Constants.AlgaeModeStates.NONE){
            runAlgaeArm(algaeArmPID.calculate(algaeArmEncoder.getPosition(), state));
        }
    }
}

// Let the rookies try to finish the code