package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public static final SparkMax elevatorMotor = new SparkMax(Constants.ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

    public static final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    

    public Elevator() {
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.inverted(false);
        elevatorMotor.configure(elevatorConfig,null,null);
    }

    public void runElevator(double percent){
        elevatorMotor.set(percent);
    }

    public void stop(){
        elevatorMotor.set(0);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        SmartDashboard.putNumber("ELEVATOR ENCODER", elevatorEncoder.getPosition());
    }
}
