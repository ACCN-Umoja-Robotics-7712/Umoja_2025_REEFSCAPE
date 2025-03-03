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

import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

// Intake for the 2025 robot
public class CoralIntake extends SubsystemBase {

    SparkMax coralIntakeMotor;
    RelativeEncoder coralIntakeEncoder;  
    private final I2C.Port i2cPort = I2C.Port.kMXP;
    private double state = Constants.CoralIntakeStates.NONE;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    
    public CoralIntake(){

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.inverted(false);

        coralIntakeMotor = new SparkMax(Constants.CoralConstants.coralIntakeMotorID, MotorType.kBrushless); 
        coralIntakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        coralIntakeEncoder = coralIntakeMotor.getEncoder();
        
    }
    
    public void runIntake(double percent){
        if (hasCoralSensor()) {
            if (percent > 0) {
                coralIntakeMotor.set(0);
            } else {
                coralIntakeMotor.set(percent);
            }
        } else {
            coralIntakeMotor.set(percent);
        }
    }

    public boolean hasCoralSensor(){
        boolean hasCoral = colorSensor.getProximity() > Constants.CoralConstants.hasCoralProximity;
        return hasCoral;
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
        SmartDashboard.putNumber("Coral applied current", coralIntakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Coral current", coralIntakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake State", state);
        double diff = Math.abs(coralIntakeMotor.getAppliedOutput() - coralIntakeMotor.getOutputCurrent()); //TODO: Check if applied is in amps
        SmartDashboard.putNumber("Coral diff", diff);
        boolean hasCoral = diff < Constants.CoralConstants.coralCurrentDiff && coralIntakeMotor.getOutputCurrent() != 0;
        SmartDashboard.putBoolean("hasCoral", hasCoral);
        SmartDashboard.putNumber("hasCoral color sensor", colorSensor.getProximity());
        // if (state == Constants.CoralIntakeStates.INTAKE) {
        //     runIntake(0.05); 
        // } else if (state == Constants.CoralIntakeStates.READY || state == Constants.CoralIntakeStates.NONE) {
        //     runIntake(0);
        // } else if (state == Constants.CoralIntakeStates.SHOOTING) {
        //     runIntake(-0.1);
        // }
    }
}


// Let the rookies try to finish the code