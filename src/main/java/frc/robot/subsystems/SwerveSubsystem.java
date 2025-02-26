package frc.robot.subsystems;

import com.studica.frc.AHRS;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.*;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDegree,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveReversed,
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private TrajectoryConfig trajectoryConfig;
    private PIDController xController;
    private PIDController yController;
    private ProfiledPIDController thetaController;
    public HolonomicDriveController holonomicDriveController;
    public final Timer timer = new Timer();
    
    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, new Rotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),  
            backLeft.getPosition(),
            backRight.getPosition()
        }, new Pose2d()
    );
    
    RobotConfig config;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);

                zeroHeading();
                resetEncoders();
                // frontRight.driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                // frontRight.turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
            }catch (Exception e) {
            }
        }).start();
    
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
            System.out.println("ROBOT GAVE UP PLEASE FIX CONFIG");
            return;
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setModuleStatesFromSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(Constants.DriveConstants.kPDrive, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.DriveConstants.kPTurning, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        
        trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 3. Define PID controllers for tracking trajectory
        xController = new PIDController(AutoConstants.kPXController, 0, 0);
        yController = new PIDController(AutoConstants.kPYController, 0, 0);
        thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        holonomicDriveController = new HolonomicDriveController(xController, yController, thetaController);
    }
    // Assuming this is a method in your drive subsystem
   public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + thetaController.calculate(pose.getRotation().getDegrees(), Math.toDegrees(sample.heading))
        );

        // Apply the generated speeds
        setModuleStatesFromSpeeds(speeds);
    }

    public void resetEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading(){
        RobotContainer.wantedAngle = -1;
        gyro.reset();
    }

    public double getHeading(){
        //Changes the -180->0 to 180->360 (0 to 180 stays the same)
        double heading = Math.IEEEremainder(-gyro.getYaw(), 360);
        // double heading = (-gyro.getYaw() + 360)%180;
        if(heading < 0){
            heading = 180 + (180+heading);
        }
        SmartDashboard.putNumber("HEADING", heading);
        return heading;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        System.out.println("ODOMETRY RESET");
        poseEstimator.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<Pose2d> allPointsPublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("AllPosesArray", Pose2d.struct).publish();

    @Override
    public void periodic() {
        
        SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        
        swerveStatePublisher.set(moduleStates);
        SmartDashboard.putNumber("FL", frontLeft.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("FR", frontRight.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("BL", backLeft.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("BR", backRight.getAbsoluteEncoderDegree());

        SmartDashboard.putNumber("T FL", Math.toDegrees(frontLeft.getTurningPosition())%360);
        SmartDashboard.putNumber("T FR", Math.toDegrees(frontRight.getTurningPosition())%360);
        SmartDashboard.putNumber("T BL", Math.toDegrees(backLeft.getTurningPosition())%360);
        SmartDashboard.putNumber("T BR", Math.toDegrees(backRight.getTurningPosition())%360);
        SmartDashboard.putNumber("YAW", gyro.getYaw());

        poseEstimator.update(Rotation2d.fromDegrees(-gyro.getYaw()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
        
        String tagLeftLimelightName = Constants.LimelightConstants.tagName;
        String tagRightLimelightName = Constants.LimelightConstants.gamePieceName;
        if (LimelightHelpers.getTargetCount(tagLeftLimelightName) != 0 && RobotContainer.gameState == GameConstants.Robot) {
            Pose3d targetPose3d = LimelightHelpers.getTargetPose3d_RobotSpace(tagLeftLimelightName);
            Double targetYaw = targetPose3d.getRotation().getMeasureAngle().baseUnitMagnitude();
            Double targetX = targetPose3d.getX();
    
            SmartDashboard.putNumber("target yaw", targetYaw);
            SmartDashboard.putNumber("target x", targetX);

            // lined up angle perfectly (turn off limelight)
            // TODO: Move this number to constants file
            Boolean alignedYaw = targetYaw <= 0.25 || (targetYaw >= 59.75 && targetYaw <= 60.25);
            Boolean alignedX = Math.abs(targetX) <= 0.02;
            if (alignedX && alignedYaw){
                // TODO: Replace with LEDs ready for game
                LimelightHelpers.setLEDMode_ForceOff(tagRightLimelightName);
            } else {
                // TODO: Replace with LEDs not game ready
                LimelightHelpers.setLEDMode_ForceOn(tagRightLimelightName);
            }
        } else {
            // TODO: Replace with LEDs not game ready
            LimelightHelpers.setLEDMode_ForceOn(tagRightLimelightName);
        }
       
        LimelightHelpers.SetRobotOrientation(tagLeftLimelightName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate leftMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tagLeftLimelightName);
        LimelightHelpers.PoseEstimate rightMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tagRightLimelightName);

        boolean doRejectUpdate = false;

        if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        double visionTrustValue = 0.1;
        if (RobotContainer.gameState == GameConstants.Robot) {
            visionTrustValue = 0;
        }
        if (leftMT2 != null) {
            if (leftMT2.tagCount == 0)
            {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate)
            {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustValue,visionTrustValue,9999999));
                poseEstimator.addVisionMeasurement(
                    leftMT2.pose,
                    leftMT2.timestampSeconds);
            }
        }
        if (rightMT2 != null) {
            if (rightMT2.tagCount == 0)
            {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate)
            {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustValue,visionTrustValue,9999999));
                poseEstimator.addVisionMeasurement(
                    rightMT2.pose,
                    rightMT2.timestampSeconds);
            }
        }
    
        posePublisher.set(poseEstimator.getEstimatedPosition());
        publishRobotPositions();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    public void setModuleStatesFromSpeeds(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    public Trajectory getNearestTagTrajectory(boolean faceCoral, boolean faceProcessor) {
        Pose2d nearestPoint = nearestPoint(faceCoral, faceProcessor);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            RobotContainer.swerveSubsystem.poseEstimator.getEstimatedPosition(),
            List.of(),
            nearestPoint,
            trajectoryConfig);
        return trajectory;
    }

    public void publishRobotPositions() {
        ArrayList<Pose2d> allPoints = new ArrayList<>();
        for (Pose2d point: Constants.bluePickUpPositions) {
            allPoints.add(point);
            allPoints.add(offsetPoint(point, 3*Constants.Measurements.coralStationDivotOffset));
            allPoints.add(offsetPoint(point, -3*Constants.Measurements.coralStationDivotOffset));
        }
        for (Pose2d point: Constants.blueReefPositions) {
            allPoints.add(point);
            allPoints.add(offsetPoint(point, Constants.Measurements.branchOffset));
            allPoints.add(offsetPoint(point, -Constants.Measurements.branchOffset));
        }
        allPoints.add(Constants.blueProcessorPosition);
        
        for (Pose2d point: Constants.redPickUpPositions) {
            allPoints.add(point);
            allPoints.add(offsetPoint(point, 3*Constants.Measurements.coralStationDivotOffset));
            allPoints.add(offsetPoint(point, -3*Constants.Measurements.coralStationDivotOffset));
        }
        for (Pose2d point: Constants.redReefPositions) {
            allPoints.add(point);
            allPoints.add(offsetPoint(point, Constants.Measurements.branchOffset));
            allPoints.add(offsetPoint(point, -Constants.Measurements.branchOffset));
        }
        allPoints.add(Constants.redProcessorPosition);
        allPointsPublisher.set(allPoints.toArray(new Pose2d[0]));
        // To-do: print points in terminal to put into choreo 
        for (Pose2d point: allPoints){
            // System.out.print(point);
        }
    }

    public Pose2d nearestPoint(boolean faceReef, boolean faceProcessor) {
        boolean isBlue = true;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                isBlue = false;
            }
        }

        if (faceProcessor) {
            if (isBlue) {
                return Constants.blueProcessorPosition;
            } else {
                return Constants.redProcessorPosition;
            }
        }

        List<Pose2d> pointsToCheck;
        if (faceReef) {
            if (isBlue) {
                pointsToCheck = List.of(Constants.blueReefPositions);
            } else {
                pointsToCheck = List.of(Constants.redReefPositions);
            }
        } else {
            if (isBlue) {
                pointsToCheck = List.of(Constants.bluePickUpPositions);
            } else {
                pointsToCheck = List.of(Constants.redPickUpPositions);
            }
            
        }
        return poseEstimator.getEstimatedPosition().nearest(pointsToCheck);
    }

    public Pose2d offsetPoint(Pose2d pose, double offset) {
        Transform2d transform = new Transform2d(0, offset, new Rotation2d(0));
        return pose.transformBy(transform);
    }

}
