package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Colors;
import frc.robot.Constants.CoralIntakeStates;
import frc.robot.Constants.GameConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.ColorSensorV3;

// Intake for the 2025 robot
public class CoralIntake extends SubsystemBase {

    SparkMax coralIntakeMotor;
    RelativeEncoder coralIntakeEncoder;  
    private double state = Constants.CoralIntakeStates.NONE;
    public static final DigitalInput intakeSensor = new DigitalInput(1);
    private double prevHasCoral = -1;
    private Color intakeColor;
    public boolean hasCoral;
    public double intakePercent = 0;
    
    public CoralIntake(){

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.inverted(false);

        coralIntakeMotor = new SparkMax(Constants.CoralConstants.coralIntakeMotorID, MotorType.kBrushless); 
        coralIntakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
        
        coralIntakeEncoder = coralIntakeMotor.getEncoder();
        
    }
    
    public void runIntake(double percent){
        this.intakePercent = percent;
        if (hasCoralSensor()) {
            if (percent > 0) {
                coralIntakeMotor.set(percent * 0.05);
            } else {
                coralIntakeMotor.set(percent);
            }
        } else {
            coralIntakeMotor.set(percent);
        }
    }

    public boolean hasCoralSensor(){
        boolean hasCoral = !intakeSensor.get();
        // dropped/shoot coral
        if (!hasCoral && Math.abs(prevHasCoral - Timer.getTimestamp()) < 1.5) {
            return true;
        } else {
            if (hasCoral) {
                prevHasCoral = Timer.getTimestamp();
            } else {
                prevHasCoral = -1;
            }
        }
        return hasCoral;
    }

    public boolean isRunning() {
        return intakePercent != 0;
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
        SmartDashboard.putBoolean("hasCoral", hasCoralSensor());
        SmartDashboard.putBoolean("hasCoral color sensor", !intakeSensor.get());

        if(isRunning() && RobotContainer.gameState==GameConstants.TeleOp){
            if(hasCoralSensor()){
                intakeColor = Color.kPurple;
            } else {
                intakeColor = Colors.white;
            }
            RobotContainer.led.setLEDColor(intakeColor);
        }
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