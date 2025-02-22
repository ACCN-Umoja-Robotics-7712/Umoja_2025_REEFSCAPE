// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {   

    public final class USB{
        public static final int DRIVER_CONTROLLER = 0;      // Driver Controller USB ID
        public static final int OPERATOR_CONTROLLER = 1;    // Operator controller USB ID
        public static final int OPERATOR_LY = 1;
        public static final int OPERATOR_LX = 0;
        public static final int OPERATOR_RY = 5;
        public static final int OPERATOR_RX = 4;
        public static final int OPERATOR_RT = 3;
        public static final int OPERATOR_LT = 2;
    }

    public final class ModuleConstants{
        public static final double kWheelDiameterMeters = 0.10;
        // gear ratio from thrifty swerve https://thethriftybot.com/products/thrify-swerve gear ratio options (pinion size 12 + second stage gear 16t? (only confirmed pinion))
        public static final double kDriveMotorGearRatio = 1/6.0;
        public static final double kTurningMotorGearRatio = 1 / 21.42857142857143;//(12*14)/(72*50) based on #of teeth
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurnEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRPM2RadPerSec = kTurnEncoderRot2Rad / 60;
        public static final double kPTurning = 0.6;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22);
        // Distance between front and back wheels

        public static final double kRobotRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 16;
        public static final int kFrontLeftTurningMotorPort = 15;

        public static final int kBackLeftDriveMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 11;

        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kFrontRightTurningMotorPort = 21;

        public static final int kBackRightDriveMotorPort = 26;
        public static final int kBackRightTurningMotorPort = 25;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kBackRightDriveReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        // OLD OFFSETS
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRot = 0.849;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRot = 0.597;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRot = 0.148;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRot = 0.613;

        // OFFSETS
        // If robot is positively off, subtract
        // Else, add
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegree = 61.645715;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDegree = 212.642091;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDegree = 142.585028;
        public static final double kBackRightDriveAbsoluteEncoderOffsetDegree = 189.405267;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSlowButtonDriveModifier = 0.2;
        public static final double kSlowButtonTurnModifier = 0.5;
        public static final double kPDrift = 0.1;
    }


    public static final class RobotStates{

        public static final int NONE = -1;
        public static final int PICKUP = 0;
        public static final int L1 = 1;
        public static final int L2 = 2;
        public static final int L3 = 3;
        public static final int L4 = 4; 
        public static final int CLIMB_READY = 5;
        public static final int CLIMBING = 6;
        public static final int REMOVE = 7;
    }

    public static final class DeepClimbStates{
        
        public static final double NONE = 01; 
        public static final double ZERO = 0; 
        public static final double READY = 0; // encoder value
        public static final double CLIMBING = 0; // encoder value
    }

    public static final class DeepClimbConstants{

        public static final int climbMotorID = 3;
        public static final double kP = 0;
        public static final double kI = 0;
    }
    public static final class AlgaeModeStates{

        public static final double NONE = -1;
        public static final double PICKUP = 0;
        public static final double READY = 1;
        public static final double SHOOTING = 2;
    }

    public static final class CoralConstants {

        public static final int coralIntakeMotorID = 1;
        public static final int coralArmMotorID = 2;

        public static final double coralArmBottomLimit = 0.0;
        public static final double coralArmTopLimit = 3.0;

        public static final double kP = 0.1;
        
        public static final double armPickupPosition = 0;

        public static final double shootPositionL1 = 0;
        public static final double shootPositionL23 = 0;
        public static final double shootPositionL4 = 0;

        public static final double coralCurrent = 0;
    }

    public static final class CoralArmStates {
        
        public static final double PICKUP = 0;
        public static final double L1 = 0;
        public static final double L23 = 0;
        public static final double L4 = 0;
        public static final double CLIMB = 0;
        public static final double REMOVE = 0;
    }

    public static final class CoralIntakeStates {
        
        public static final double NONE = -1;
        public static final double INTAKE = 1;
        public static final double READY = 2;
        public static final double SHOOTING = 3;

    }

    public static final class AlgaeConstants {

        public static final int algaeIntakeMotorID = 3;
        public static final int algaeArmMotorID = 4; 
        public static final double kP = 0.1;
        public static final double armPickupPosition = 0;
        public static final double armLimiter = 0.05;
    }

    public static final class AlgaeArmStates {

        public static final double PICKUP = 0.0;
        public static final double READY = 0;
        public static final double SHOOTING = 0;
    }

    public static final class ElevatorConstants {
        public static final int leftMotorID = 50;
        public static final int rightMotorID = 51;

        public static final double elevatorTopLimit = 5.0;
        public static final double elevatorBottomLimit = 0.0;

        public static final double kP =  0.1;

        public static final int elevatorMotor1ID = 8; // Check this
        public static final int elevatorMotor2ID = 9; // Check this

    }

    public static final class ElevatorStates {

        public static final double NONE = -1;
        public static final double L1 = 0;
        public static final double L2 = 0;
        public static final double L3 = 0;
        public static final double L4 = 0;
        public static final double CLIMB = 0;
    };

    public static final class LEDConstants {
        public static final int numLEDsPerStrip = 36;
        public static final int numLEDsPerColor = 3;
    }

    public static final class GameConstants {
        public static final int Robot = 0;
        public static final int Auto = 1;
        public static final int TeleOp = 2;
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final int kDriverRB = 6;

        public static final double kDeadband = 0.05;

        //BUTTONS

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int L3 = 9;
        public static final int R3 = 10;

        //AXES

        public static final int LX = 0;
        public static final int LY = 1;
        public static final int LT = 2;

        public static final int RT = 3;
        public static final int RX = 4;
        public static final int RY = 5;
    }

    public static final class LimelightConstants {
        public static final String tagName = "limelight-tags";
        public static final String gamePieceName = "limelight-gp";
        public static final int Estimate_Distance = 20;
    }

    public static final class Colors {
        public static final Color red = Color.kRed;
        public static final Color blue = Color.kBlue;
        public static final Color green = new Color(0, 255, 0);
        public static final Color white = Color.kWhite;
        public static final Color uRed = new Color(20, 0 , 0);
        public static final Color uDarkOrange = new Color(254,17,1);
        public static final Color uGreen = new Color(0, 7, 0);
        public static final Color uOrange = new Color(255, 25, 0);
        public static final Color[] uColors = {uRed, uDarkOrange, uGreen, uOrange};
    }

    public static final class RobotPositions {
        public static final Pose2d redCenter = new Pose2d(11.62, 4.0259, new Rotation2d(0));
        public static final Pose2d redCenterSafe = new Pose2d(11.35, 4.0259, new Rotation2d(0));
    }
}
