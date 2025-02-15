package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor1 = new SparkMax(Constants.ElevatorConstants.elevatorMotor1ID, MotorType.kBrushless);
    private final SparkMax elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.elevatorMotor2ID, MotorType.kBrushless);

    private final RelativeEncoder elevator1Encoder = elevatorMotor1.getEncoder();
    private final PIDController PID = new PIDController(0, 0, 0);
    
    public Elevator() {
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        SparkMaxConfig elevator2Config = new SparkMaxConfig();

        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.inverted(false);
        elevatorMotor1.configure(elevatorConfig,null,null);

        elevator2Config.idleMode(IdleMode.kBrake);
        elevator2Config.follow(elevatorMotor1.getDeviceId(), true);
        elevatorMotor2.configure(elevator2Config, null, null);
    }


    public void runElevator(double percent){
        elevatorMotor1.set(percent);
    }

    public void stop(){
        elevatorMotor1.set(0);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        SmartDashboard.putNumber("ELEVATOR ENCODER", elevator1Encoder.getPosition());
    }
}
