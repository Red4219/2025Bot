package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.mechanisms.SwerveModule;
import frc.robot.tools.Limelight;
import frc.robot.tools.PhotonVision;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.autonomous.AimCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
//import frc.robot.Constants.DriveConstants.kDriveModes;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.mechanisms.LED.LEDStatus;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;


public class DriveSubsystem extends SubsystemBase {

	//private boolean fieldRelative = true;
	private boolean gyroTurning = false;
	private double targetRotationDegrees;

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule rearLeft;
	private final SwerveModule rearRight;

	private SwerveModulePosition[] swervePosition;
	private SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
		new ChassisSpeeds(0, 0, 0)
	);

	// Initalizing the gyro sensor
	//private final AHRS gyro = new AHRS(SPI.Port.kMXP);
	private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

	private double xSpeed = 0.0;
	private double ySpeed = 0.0;
	private double rot = 0.0;

	//private kDriveModes mode = kDriveModes.NORMAL;
	//private int speakerTarget = 0;
	//private boolean targetLocked = false;
	private boolean isSim = false;
	private LimelightHelpers.PoseEstimate limelightMeasurement = null;

	// Odeometry class for tracking robot pose
	private SwerveDriveOdometry odometry;
	private boolean limeLightCanSeeTag = false;
	//private boolean photonVisionCanSeeTag = false;

	// PID controller for gyro turning
	private ProfiledPIDController gyroTurnPidController = null;

	private SwerveDrivePoseEstimator poseEstimator = null;

	private PhotonVision _photonVision = null;
	private Limelight _limeLight = null;
	private SwerveModuleState[] swerveModuleStatesRobotRelative;
	private EstimatedRobotPose phoneEstimatedRobotPose;

	private double driveP = ModuleConstants.kModuleDriveGains.kP;
	private double driveI = ModuleConstants.kModuleDriveGains.kI;
	private double driveD = ModuleConstants.kModuleDriveGains.kD;

	private double turnP = ModuleConstants.kModuleTurningGains.kP;
	private double turnI = ModuleConstants.kModuleTurningGains.kI;
	private double turnD = ModuleConstants.kModuleTurningGains.kD;

	//private Rotation2d rotationOffset180 = new Rotation2d(0);

	/*private double autoDriveP = AutoConstants.PathPLannerConstants.kPPDriveConstants.kP;
	private double autoDriveI = AutoConstants.PathPLannerConstants.kPPDriveConstants.kI;
	private double autoDriveD = AutoConstants.PathPLannerConstants.kPPDriveConstants.kD;*/

	private Trajectory trajectory = null;
	//private Trajectory.State goal = null;
	private Command pathfindingCommand = null;
	private boolean gotoPoseRunning = false;


	// test for auto positioning
	HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
		new PIDController(1, 0, 0),
		new PIDController(1, 0, 0),
		new ProfiledPIDController(
			2,
			0,
			0,
			new TrapezoidProfile.Constraints(
				5.0,
				5.0
			)
		)
	);

	/**
	* Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
    * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
    */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
    /**
    * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(
		Constants.PhotonVisionConstants.visionMeasurementStdDevsX, 
		Constants.PhotonVisionConstants.visionMeasurementStdDevsY, 
		Constants.PhotonVisionConstants.visionMeasurementStdDevsTheta
	);

	public AHRS getGyro() {
		return this.gyro;
	}

	//private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {

		if(Constants.kEnablePhotonVision) {
			_photonVision = RobotContainer.photonVision;
		}

		if(Constants.kEnableLimelight) {
			_limeLight = RobotContainer.limelight;
		}

		if(RobotBase.isReal()) {
			isSim = false;
		} else {
			isSim = true;
		}
		
		gyro.reset();
		//_gyroIONavX = new GyroIONavX(gyro);

		frontLeft = new SwerveModule(
				"FL",
				ModuleConstants.kFrontLeftDriveMotorPort,
				ModuleConstants.kFrontLeftTurningMotorPort,
				ModuleConstants.kFrontLeftTurningEncoderPort,
				ModuleConstants.kFrontLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				true

			);

		frontRight = new SwerveModule(
				"FR",
				ModuleConstants.kFrontRightDriveMotorPort,
				ModuleConstants.kFrontRightTurningMotorPort,
				ModuleConstants.kFrontRightTurningEncoderPort,
				ModuleConstants.kFrontRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				true
			);

		rearLeft = new SwerveModule(
				"RL",
				ModuleConstants.kRearLeftDriveMotorPort,
				ModuleConstants.kRearLeftTurningMotorPort,
				ModuleConstants.kRearLeftTurningEncoderPort,
				ModuleConstants.kRearLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				true
			);

		rearRight = new SwerveModule(
				"RR",
				ModuleConstants.kRearRightDriveMotorPort,
				ModuleConstants.kRearRightTurningMotorPort,
				ModuleConstants.kRearRightTurningEncoderPort,
				ModuleConstants.kRearRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains,
				true,
				true
			);

		swervePosition = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};

		odometry = new SwerveDriveOdometry(
				DriveConstants.kDriveKinematics,
				//gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition);

		gyroTurnPidController = new ProfiledPIDController(
				DriveConstants.kGyroTurningGains.kP,
				DriveConstants.kGyroTurningGains.kI,
				DriveConstants.kGyroTurningGains.kD,
				new TrapezoidProfile.Constraints(
						DriveConstants.kMaxTurningVelocityDegrees,
						DriveConstants.kMaxTurningAcceleratonDegrees));

		gyroTurnPidController.enableContinuousInput(-180, 180);
		gyroTurnPidController.setTolerance(DriveConstants.kGyroTurnTolerance);

		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				//gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition,
				new Pose2d(),
				stateStdDevs,
				visionMeasurementStdDevs
			);
		
		targetRotationDegrees = 0;

		if(Constants.kDebugDriveTrain == true) {

			// auto tab stuff
			ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
			/*autoTab.addDouble("AutoX Position", this::getAutoX_Position);
			autoTab.addDouble("AutoY Position", this::getAutoY_Position);
			autoTab.addBoolean("AutoX Status", this::getAutoPositionStatusX);
			autoTab.addBoolean("AutoY Status", this::getAutoPositionStatusY);
			autoTab.addString("Alliance", this::getAlliance);*/

			// gyro tab stuff
			ShuffleboardTab gyroTab = Shuffleboard.getTab("Gyro");
			gyroTab.addDouble("Yaw", gyro::getYaw);
			gyroTab.addDouble("Pitch", gyro::getPitch);
			gyroTab.addDouble("Roll", gyro::getRoll);

			//Swerve tab stuff
			ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
			// swerveTab.addDouble("FL Absolute", frontLeft::getAbsoluteHeading);
			// swerveTab.addDouble("FR Absolute", frontRight::getAbsoluteHeading);
			// swerveTab.addDouble("RL Absolute", rearLeft::getAbsoluteHeading);
			// swerveTab.addDouble("RR Absolute", rearRight::getAbsoluteHeading);
			// swerveTab.addDouble("FL Meters", frontLeft::getDistanceMeters);
			// swerveTab.addDouble("FR Meters", frontRight::getDistanceMeters);
			// swerveTab.addDouble("RL Meters", rearLeft::getDistanceMeters);
			// swerveTab.addDouble("RR Meters", rearRight::getDistanceMeters);
			// swerveTab.addBoolean("Auto Aim", this::autoAim);
			// swerveTab.addBoolean("Target Locked", this::getTargetLocked);

            SmartDashboard.putData(this);
            Shuffleboard.getTab("Swerve")
				.add(this);
		}

		gyro.reset();

		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
	}

	/*public void setupAuto(boolean setupAuto) {
		this.setupAuto = setupAuto;
	}*/

	/*public void setStartPosition(Pose2d startPosition) {
		this.startPosition = startPosition;
	}*/

	@Override
	public void simulationPeriodic() {

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		updateOdometry();

		if(Constants.kDebugDriveTrain == true) {
			//SmartDashboard.putNumber("FL Offset Check", frontLeft.getAbsoluteHeading() + frontLeft.angleZero);
			//SmartDashboard.putNumber("FR Offset Check", frontRight.getAbsoluteHeading() + frontRight.angleZero);
			//SmartDashboard.putNumber("RL Offset Check", rearLeft.getAbsoluteHeading() + rearLeft.angleZero);
			//SmartDashboard.putNumber("RR Offset Check", rearRight.getAbsoluteHeading() + rearRight.angleZero);
			SmartDashboard.putNumber("2D X", getPose().getX());
			SmartDashboard.putNumber("2D Y", getPose().getY());
			SmartDashboard.putNumber("2D Gyro", -odometry.getPoseMeters().getRotation().getDegrees());
			//SmartDashboard.putData("field", RobotContainer.field);
		}

		SmartDashboard.putData("field", RobotContainer.field);

		//Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());

		// Show the estimated position
		//estimatedPose = poseEstimator.getEstimatedPosition();
		//Logger.recordOutput("Estimator/Robot", estimatedPose);

		if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());
		}

		/*combinedEstimatedPoseArray[0] = estimatedPose.getX();
		combinedEstimatedPoseArray[1] = estimatedPose.getY();
		combinedEstimatedPoseArray[2] = estimatedPose.getRotation().getDegrees();
		Logger.recordOutput("Estimator/PoseArray", combinedEstimatedPoseArray);*/
	}

	// region getters
	public double getHeading() {
		if(isSim) {
			//System.out.println("getHeading is: " + gyro.getRotation2d().getDegrees());
			return gyro.getRotation2d().getDegrees();
		}
		//return gyro.getRotation2d().unaryMinus().getDegrees();
		return gyro.getRotation2d().getDegrees();
	}

	public double getHeading360() {
		if(isSim) {
			return (gyro.getRotation2d().getDegrees() % 360);
		}
		//return (gyro.getRotation2d().unaryMinus().getDegrees() % 360);
		return (gyro.getRotation2d().getDegrees() % 360);
	}

	public double getRoll() {
		return gyro.getRoll();
	}

	public double getPitch() {
		return gyro.getPitch();
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public Pose2d getPoseEstimatorPose2d() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {

		odometry.resetPosition(
			gyro.getRotation2d(),
			swervePosition,
			pose
		);

		poseEstimator.resetPosition(
			gyro.getRotation2d(),
			swervePosition,
			pose
		);

		if(Constants.kEnablePhotonVision) {
			_photonVision.setReferencePose(pose);
		}
	}

	public void drive(double xSpeed, double ySpeed, double rot) {

		// Apply deadbands to inputs
		xSpeed *= ModuleConstants.kMaxModuleSpeedMetersPerSecond;
		ySpeed *= ModuleConstants.kMaxModuleSpeedMetersPerSecond;

		// if (gyroTurning) {
		// //if(gyro.isRotating()) {
		// 	targetRotationDegrees += rot;
		// 	rot = gyroTurnPidController.calculate(getHeading360(), targetRotationDegrees);
		// } else {
		// 	rot *= DriveConstants.kMaxRPM;
		// }

		rot *= DriveConstants.kMaxRPM;

		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.rot = rot;

		if(isSim) {
			swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				//ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.poseEstimator.getEstimatedPosition().getRotation())
			);

			// testing
			// int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
			// SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			// angle.set(
			// 	angle.get() 
			// 	+ ChassisSpeeds.fromFieldRelativeSpeeds(
			// 		xSpeed, 
			// 		ySpeed, 
			// 		rot, 
			// 		gyro.getRotation2d()
			// 	).omegaRadiansPerSecond
			// );
			// end testing
		} else {

			swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				//ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d().unaryMinus())
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
			);
		}
		
		setModuleStates(swerveModuleStates);
	}

	public void driveRobotRelative(double xSpeed, double ySpeed, double rot) {
		// Apply deadbands to inputs
		xSpeed *= ModuleConstants.kMaxModuleSpeedMetersPerSecond;
		ySpeed *= ModuleConstants.kMaxModuleSpeedMetersPerSecond;

		if (gyroTurning) {
			targetRotationDegrees += rot;
			rot = gyroTurnPidController.calculate(getHeading360(), targetRotationDegrees);
		} else {
			rot *= DriveConstants.kMaxRPM;
		}

		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.rot = rot;

		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

		swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
			chassisSpeeds
		);

		setModuleStates(swerveModuleStates);
	}

	// This is for auto
	public ChassisSpeeds getChassisSpeedsRobotRelative() {

		if(isSim) {
			return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
		} 
		
		//return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d().unaryMinus());
		//return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
		return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, poseEstimator.getEstimatedPosition().getRotation());
	}

	// This is for auto
	public DriveFeedforwards setChassisSpeedsRobotRelative(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedForwards ) {

		// chassisSpeeds = chassisSpeeds.times(-1.0);
		
		swerveModuleStatesRobotRelative = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		// In simulation, the actual navx does not work, so set the value from the chassisSpeeds
		if(isSim) {
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
			SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			angle.set(angle.get() + -chassisSpeeds.omegaRadiansPerSecond);
		}

		frontLeft.setDesiredState(swerveModuleStatesRobotRelative[0]);
		frontRight.setDesiredState(swerveModuleStatesRobotRelative[1]);
		rearLeft.setDesiredState(swerveModuleStatesRobotRelative[2]);
		rearRight.setDesiredState(swerveModuleStatesRobotRelative[3]);

		if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("SwerveModuleStates/Setpoints", swerveModuleStatesRobotRelative);
		}

		return feedForwards;
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		
		if(isSim) {

			ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
				desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
			);

			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[4]");
			SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			angle.set(angle.get() + -chassisSpeeds.omegaRadiansPerSecond);
		}

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[3]);

		if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("SwerveModuleStates/Setpoints", desiredStates);
		}
	}

	public void updateOdometry() {
		swervePosition[0] = frontLeft.getPosition();
		swervePosition[1] = frontRight.getPosition();
		swervePosition[2] = rearLeft.getPosition();
		swervePosition[3] = rearRight.getPosition();

		if (Constants.kEnablePhotonVision) {

			phoneEstimatedRobotPose = _photonVision.getPose(poseEstimator.getEstimatedPosition());

			if(phoneEstimatedRobotPose != null) {
				if(Constants.kEnableDriveSubSystemLogger) {
					Logger.recordOutput("PhotonVisionEstimator/Robot", phoneEstimatedRobotPose.estimatedPose.toPose2d());
				}

				poseEstimator.addVisionMeasurement(
					phoneEstimatedRobotPose.estimatedPose.toPose2d(),
					//Timer.getFPGATimestamp() - phoneEstimatedRobotPose.timestampSeconds,
					phoneEstimatedRobotPose.timestampSeconds,
					visionMeasurementStdDevs
				);

				if(!gyro.isMoving() && Constants.kResetOdometryFromPhotonVision && !isSim) {
					// if we are not moving, reset the odometry to the location from the limelight
					//resetOdometry(limelightMeasurement.pose.rotateBy(rotationOffset180));
					
					// this should already be rotated above
					resetOdometry(phoneEstimatedRobotPose.estimatedPose.toPose2d());
				}
			}
		}

		if (Constants.kEnableLimelight) {
			//
			//limelightMeasurement = _limeLight.getPose2d(poseEstimator.getEstimatedPosition().rotateBy(rotationOffset180));
			limelightMeasurement = _limeLight.getPose2d(poseEstimator.getEstimatedPosition());
			// System.out.println(limelightMeasurement.tagCount);
			// Did we get a measurement?
			if(limelightMeasurement != null && limelightMeasurement.tagCount >= 1) {
				limeLightCanSeeTag = true;

     			poseEstimator.addVisionMeasurement(
         			limelightMeasurement.pose,
         			limelightMeasurement.timestampSeconds
				);

				if(Constants.kEnableDriveSubSystemLogger) {
					Logger.recordOutput("Limelight/Pose", limelightMeasurement.pose);
				}


				if(!gyro.isMoving() && Constants.kResetOdometryFromLimeLight) {
					// if we are not moving, reset the odometry to the location from the limelight
					//resetOdometry(limelightMeasurement.pose.rotateBy(rotationOffset180));
					
					// this should already be rotated above
					resetOdometry(limelightMeasurement.pose);
				}
			} else {
				//RobotContainer.led1.setStatus(LEDStatus.targetSearching);
				limeLightCanSeeTag = false;
			}

		}


		if(isSim) {

			odometry.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition);

			poseEstimator.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition
			);

		} else {
			odometry.update(
				//gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition
			);

			/*estimatedPose = poseEstimator.update(
				gyro.getRotation2d().unaryMinus(),
				swervePosition
			);*/
			poseEstimator.update(
				//gyro.getRotation2d().unaryMinus(),
				gyro.getRotation2d(),
				swervePosition
			);
		}

		// Show the estimated position
		//Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());

		if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());
		}

		// Update the field with the location of the robot
		//RobotContainer.field.setRobotPose(odometry.getPoseMeters());
		RobotContainer.field.setRobotPose(poseEstimator.getEstimatedPosition());

		// this needs to be fixed to show if we are in the area of the selected auto position
		/*if(setupAuto && startPosition != null) {
			RobotContainer.led1.setStatus(LEDStatus.problem);
		} else if(setupAuto && startPosition == null) {
			RobotContainer.led1.setStatus(LEDStatus.problem);
		}*/

		/*if(photonVisionCanSeeTag || limeLightCanSeeTag) {
			RobotContainer.led1.setStatus(LEDStatus.targetAquired);
		} else {
			RobotContainer.led1.setStatus(LEDStatus.problem);
		}*/
	}

	/*public void updateOdometrySim() {

	}*/

	public void resetEncoders() {
		frontLeft.resetEncoders();
		rearLeft.resetEncoders();
		frontRight.resetEncoders();
		rearRight.resetEncoders();
	}

	public void zeroHeading() {
		gyro.reset();
	}

	public void stopMotors() {
		frontLeft.stopMotors();
		frontRight.stopMotors();
		rearLeft.stopMotors();
		rearRight.stopMotors();
	}

	public String getAlliance() {
		String alliance = "";
		if(DriverStation.getAlliance().isPresent()) {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				alliance = "Blue";
			} else {
				alliance = "Red";
			}
		}

		return alliance;
	}

	public double getDriveP() {
		return driveP;
	}

	public double getDriveI() {
		return driveI;
	}

	public double getDriveD() {
		return driveD;
	}

	public void setDriveP(double p) {
		driveP = p;

		this.frontLeft.setDrivePID(driveP, driveI, driveD);
		this.frontRight.setDrivePID(driveP, driveI, driveD);
		this.rearLeft.setDrivePID(driveP, driveI, driveD);
		this.rearRight.setDrivePID(driveP, driveI, driveD);
	}

	public void setDriveI(double i) {
		driveI = i;

		this.frontLeft.setDrivePID(driveP, driveI, driveD);
		this.frontRight.setDrivePID(driveP, driveI, driveD);
		this.rearLeft.setDrivePID(driveP, driveI, driveD);
		this.rearRight.setDrivePID(driveP, driveI, driveD);
	}

	public void setDriveD(double d) {
		driveD = d;

		this.frontLeft.setDrivePID(driveP, driveI, driveD);
		this.frontRight.setDrivePID(driveP, driveI, driveD);
		this.rearLeft.setDrivePID(driveP, driveI, driveD);
		this.rearRight.setDrivePID(driveP, driveI, driveD);
	}

	public double getTurnP() {
		return turnP;
	}

	public double getTurnI() {
		return turnI;
	}

	public double getTurnD() {
		return turnD;
	}

	public void setTurnP(double p) {
		turnP = p;
	}

	public void setTurnI(double i) {
		turnI = i;
	}

	public void setTurnD(double d) {
		turnD = d;
	}

	public double getAutoDriveP() {
		return AutoConstants.PathPLannerConstants.kPPDriveConstants.kP;
	}

	public double getAutoDriveI() {
		return AutoConstants.PathPLannerConstants.kPPDriveConstants.kI;
	}

	public double getAutoDriveD() {
		return AutoConstants.PathPLannerConstants.kPPDriveConstants.kD;
	}
	
	public void CreateAutoBuilder() {

		try {

			RobotConfig config = RobotConfig.fromGUISettings();
			//PIDConstants kPPDriveConstants = new PIDConstants(autoDriveP, autoDriveI, autoDriveD);

			AutoBuilder.configure(
				this::getPoseEstimatorPose2d, 
				this::resetOdometry, 
				this::getChassisSpeedsRobotRelative,
				this::setChassisSpeedsRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards 
				new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
					AutoConstants.PathPLannerConstants.kPPDriveConstants, // Translation PID constants
					AutoConstants.PathPLannerConstants.kPPTurnConstants // Rotation PID constants
            	),
				config, 
				this::getAllianceAuto,
				this
			);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private boolean getAllianceAuto() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			return alliance.get() == DriverStation.Alliance.Red;
		}
		return false;
	}

	@Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        /* builder.addDoubleProperty("Turn_D", this::getTurnD, this::setTurnD);
        builder.addDoubleProperty("Turn_I", this::getTurnI, this::setTurnI);
        builder.addDoubleProperty("Turn_P", this::getTurnP, this::setTurnP);
        builder.addDoubleProperty("Drive_D", this::getDriveD, this::setDriveD);
        builder.addDoubleProperty("Drive_I", this::getDriveI, this::setDriveI);
        builder.addDoubleProperty("Drive_P", this::getDriveP, this::setDriveP);
		builder.addDoubleProperty("Auto_Drive_P", this::getAutoDriveP, null);
		builder.addDoubleProperty("Auto_Drive_I", this::getAutoDriveI, null);
		builder.addDoubleProperty("Auto_Drive_D", this::getAutoDriveD, null); */
		
    }

	public Trajectory.State generateTrajectory(Pose2d targetPose) {

		var sideStart = new Pose2d(
			odometry.getPoseMeters().getX(), 
			odometry.getPoseMeters().getY(),
			odometry.getPoseMeters().getRotation()
		);
			
		var interiorWaypoints = new ArrayList<Translation2d>();
	
		TrajectoryConfig config = new TrajectoryConfig(
			Units.feetToMeters(12),
			Units.feetToMeters(12)
		);

		try {
	
			trajectory = TrajectoryGenerator.generateTrajectory(
				sideStart,
				interiorWaypoints,
				//crossScale,
				targetPose,
				config
			);
		} catch (Exception e) {
			//System.out.println("DriveSubsystem::generateTrajectory() - " + e.getMessage());
			return null;
		}

		return trajectory.sample(trajectory.getTotalTimeSeconds());
	}

	public void goToPose(Constants.PoseDefinitions.kFieldPoses targetPose) {

		//System.out.println("DriveSubsystem::goToPose() called");

		// If it is running and we call it again, cancel it
		/*if(gotoPoseRunning) {
			pathfindingCommand.cancel();
			gotoPoseRunning = false;
			return;
		}*/

		// Since we are using a holonomic drivetrain, the rotation component of this pose
		// represents the goal holonomic rotation
		Pose2d targettPose = new Pose2d(5.973, 0.672, Rotation2d.fromDegrees(-90));

		// Create the constraints to use while pathfinding
		PathConstraints constraints = new PathConstraints(
        	1.5, 4.0,
        	Units.degreesToRadians(540), Units.degreesToRadians(720)
		);


		if(DriverStation.getAlliance().isPresent()) {
			if(DriverStation.getAlliance().get() == Alliance.Blue) {
				// Since AutoBuilder is configured, we can use it to build pathfinding commands
				pathfindingCommand = AutoBuilder.pathfindToPose(
					targettPose,
					constraints,
					0.0
				);

				//pathfindingCommand.andThen(new AimCommand(_photonVision, 3)).finallyDo(this::gotoPoseRunning).schedule();
				pathfindingCommand = pathfindingCommand.andThen(new AimCommand(_photonVision, 3)).finallyDo(this::gotoPoseRunning);
				pathfindingCommand.schedule();

				//pathfindingCommand.schedule();
			} else {
				// Since AutoBuilder is configured, we can use it to build pathfinding commands
				pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(
					targettPose,
					constraints,
					0.0
				);

				//pathfindingCommand.andThen(new AimCommand(_photonVision, 3)).finallyDo(this::gotoPoseRunning).schedule();
				pathfindingCommand = pathfindingCommand.andThen(new AimCommand(_photonVision, 3)).finallyDo(this::gotoPoseRunning);
				pathfindingCommand.schedule();
			}
		} else {
			System.out.println("DriveSubsystem::gotoPose() called, but DriverStation.getAlliance() is null");
		}
	}

	public void gotoPoseCancel() {
		if(pathfindingCommand != null) {
			pathfindingCommand.cancel();
			pathfindingCommand.end(true);

			//System.out.println("DriveSubsystem::gotoPoseCancel() setting it to false");
			gotoPoseRunning = false;
		}
	}

	public void gotoPoseRunning(boolean running) {
		//System.out.println("DriveSubsystem::gotoPoseRunning() setting it to " + running);
		gotoPoseRunning = running;
	}

	public boolean isGotoPoseRunning() {
		return gotoPoseRunning;
	}
}
