// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.tools.parts.PIDGains;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	// Drive train
	public static boolean kDebugDriveTrain = true;
	public static boolean kEnableDriveSubSystemLogger = true;
	public static boolean enableLogger = false;

	// Photonvision
	public static boolean kEnablePhotonVision = false;
	public static boolean kDebugPhotonVision = false;

	// Limelight
	public static boolean kEnableLimelight = true;
	public static boolean kDebugLimelight = true;
	
	// Arm
	public static boolean kEnableArm = true;
	public static boolean kEnableDebugArm = true;

	// Elevator
	public static boolean kEnableElevator = true;
	public static boolean kEnableDebugElevator = true;

	// End Effector
	public static boolean kEnableEndEffector = true;
	public static boolean kEnableDebugEndEffector = true;

	public static final String kRioCANBusName = "rio";
	public static final String kCanivoreCANBusName = "canivore";
	//public static final String logFolders = "/media/sda2/";

	public static class ModuleConstants {

		// Current limits for the wheels
		//public static final int kTurnMotorCurrentLimit = 25;
		//public static final int kDriveMotorCurrentLimit = 35;

		// Constants set for the _SDS MK4i_ and MK4
		//public static final double kdriveGearRatioL1 = 1d / 8.14;
		//public static final double kdriveGearRatioL2 = 1d / 6.75;
		public static final double kdriveGearRatioL3 = 1d / 6.12;
		//public static final double kdriveGearRatioL3 = 1d / 5.7;
		//public static final double kdriveGearRatioL4 = 1d / 5.14;
		public static final double kturnGearRatio = 1d / (150d / 7d);

		public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;

		// The max speed the modules are capable of
		//public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(16.5);
		public static final double kMaxModuleSpeedMetersPerSecond = 16.5;

		//public static final double ksVolts = .1;
		//public static final double kDriveFeedForward = .2;

		// TODO: Retune feedforward values for turning
		public static final double kvTurning = .43205;
		public static final double ksTurning = .17161; 

		// NEO drive motor CAN ID's
		public static final int kFrontLeftDriveMotorPort = 2;
		public static final int kFrontRightDriveMotorPort = 4;
		public static final int kRearLeftDriveMotorPort = 8;
		public static final int kRearRightDriveMotorPort = 6;

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 1;
		public static final int kFrontRightTurningMotorPort = 3;
		public static final int kRearLeftTurningMotorPort = 7;
		public static final int kRearRightTurningMotorPort = 5;

		// CANcoder CAN ID's
		public static final int kFrontLeftTurningEncoderPort = 9;
		public static final int kFrontRightTurningEncoderPort = 10;
		public static final int kRearLeftTurningEncoderPort = 12;
		public static final int kRearRightTurningEncoderPort = 11;

		// Offset angle for absolute encoders (find this using CTRE client)
		public static final double kFrontLeftAngleZero = 0.0;
		public static final double kFrontRightAngleZero = 0.0;
		public static final double kRearLeftAngleZero = 0.0;
		public static final double kRearRightAngleZero = 0.0;

		public static final PIDGains kModuleDriveGains = new PIDGains(0.1, 0, 0);
		public static final PIDGains kModuleTurningGains = new PIDGains(5.5, 0.0, 0.0);
	}

	public static class DriveConstants {
		// this sets turning speed (keep this low)
		public static final double kMaxRPM = 10;
		public static final double kBumperToBumperWidth = Units.inchesToMeters(27);

		public static final double kTrackWidth = Units.inchesToMeters(27); // in meters!
		public static final double kWheelBase = Units.inchesToMeters(27); // in meters!

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR

		public static final PIDGains kGyroTurningGains = new PIDGains(.025, 0, 0);
		public static final double kMaxTurningVelocityDegrees = 20;
		public static final double kMaxTurningAcceleratonDegrees = 10;
		public static final double kGyroTurnTolerance = 2;

		public static double kAutoAlignSpeed = 0.02;
		public static double kAutoAlignTolerance = 1.0;
		public static double kAutoAlignOffset = 4.81;
		//public static double kAutoAlignLeftOffset = -16.0;
		public static double kAutoAlignRightOffset = 4.81;

		public static double kAutoAlignLeftLeft = 10.7;
		public static double kAutoAlignLeftRight = 9.7;

		public static double kAutoAlignRightLeft = 0.0;
		public static double kAutoAlignRightRight = 0.0;

		// These are to measure the distnace to the target
		public static double kAutoAlignLeftYOffset = 25.0;
		public static double kAutoAlignLeftYTolerance = 4.0;

	}

	/**
	 * The constants pertaining to Autonoumus 
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			//public static final PIDConstants kPPDriveConstants = new PIDConstants(8.5, 0, 0);
			//public static final PIDConstants kPPDriveConstants = new PIDConstants(5.0, 0, 0);
			//public static final PIDConstants kPPDriveConstants = new PIDConstants(6.5, 0, 0); // best one
			public static final PIDConstants kPPDriveConstants = new PIDConstants(7.5, 0, 0);
			//public static final PIDConstants kPPTurnConstants = new PIDConstants(3.5, 0, 0);
			//public static final PIDConstants kPPTurnConstants = new PIDConstants(5.0, 0, 0);
			public static final PIDConstants kPPTurnConstants = new PIDConstants(3.5, 0, 0);

			//public static final double kPPMaxVelocity = 4.00;
			//public static final double kPPMaxAcceleration = 2.50;
			//public static final double kMaxModuleSpeed = 4.5; // Max module speed, in m/s
			//public static final double kDriveBaseRadius = 0.4; // Drive base radius in meters. Distance from robot center to furthest module.
		}

		//public static final double kAimTargetTolerance = 2.0;
	}

	/**
	 * The constants pertaining to the drive station
	 */
	public static class OperatorConstants {

		public static final double KDeadBand = .125;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;
	}

	public static class LimelightConstants {
		public static String name1 = "limelight-left";
		public static String name2 = "limelight-right";
	}

	public static class PhotonVisionConstants {

		//public static final PoseStrategy poseStrategy = PoseStrategy.AVERAGE_BEST_TARGETS;
		public static final PoseStrategy poseStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;

		public static double camDiagFOV = 170.0;
		public static double camPitch = 0.0;
		public static double camHeightOffGround = Units.inchesToMeters(24.0);
		// the side to side position of the camera relative to the robot center
		public static double camX = Units.inchesToMeters(-15.0);
		// the front to back position of the camera relative to the robot center
		public static double camY = Units.inchesToMeters(0.0);

		public static Transform3d cameraToRobot = new Transform3d(
                    new Translation3d(
						camX,
						camY,
					 	PhotonVisionConstants.camHeightOffGround
					),
					new Rotation3d(
						0,
						PhotonVisionConstants.camPitch,
						180
					)
				);

		public static final String CameraName = "cam1";

		// Simulated Vision System.
    	// Configure these to match your PhotonVision Camera,
    	// pipeline, and LED setup.
		public static double sim_camDiagFOV = camDiagFOV; // degrees - assume wide-angle camera
		public static double sim_camPitch = camPitch; // degrees
    	public static double sim_camHeightOffGround = camHeightOffGround; // meters
    	//public static double sim_maxLEDRange = 20; // meters
    	public static int sim_camResolutionWidth = 640; // pixels
    	public static int sim_camResolutionHeight = 480; // pixels
    	//public static double sim_minTargetArea = 10; // square pixels
		//public static double sim_minTargetArea = 300; // square pixels

		/**
    	* Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    	* less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    	*/
		public static double visionMeasurementStdDevsX = 0.5;
		public static double visionMeasurementStdDevsY = 0.5;
		public static double visionMeasurementStdDevsTheta = Units.degreesToRadians(10);
	}

	public static class ClimbConstants {

		public static boolean climberEnabled = true;
		public static int motor_id = 30;

		public static double climberExtendSpeed = 0.35;
		public static double climberRetractSpeed = 0.35;
		public static double P = 0.1;
		public static double I = 0.0;
		public static double D = 0.0;
		public static int servo_id = 1;

		public static int servo_id = 1;
		public static double servoClosed = 0.0;
		public static double servoOpen = 0.0;


	}
	public static class ArmConstants {
		// how high the arm goes to clear a stuck coral on the battery
		public static double ClearCoral = .103; 
		//public static double CoralL4 = -0.9;
		public static double CoralL4 = 0.526; // using bore encoder
		//public static double CoralL3 = -2.738;
		public static double CoralL3 = 0.55; // using bore encoder
		//public static double CoralL2 = -0.9;
		public static double CoralL2 = 0.383; // using bore encoder
		//public static double CoralL1 = -10.0;
		public static double CoralL1 = 0.300; // using bore encoder
		//public static double CoralHuman = -0.462;
		public static double CoralHuman = 0.094; // using bore encoder
		//public static double AlgaeL1 = -0.9;
		public static double AlgaeL1 = 0.103; // using bore encoder
		//public static double AlgaeL2 = -0.9;
		public static double AlgaeL2 = 0.103; // using bore encoder;
		//public static double AlgaeL3 = -0.9;
		public static double AlgaeL3 = 0.103; // using bore encoder;
		//public static double AlgaeShoot = -0.9;
		public static double AlgaeShoot = 0.103; // using bore encoder
		//public static double AlgaeHuman = -0.9;
		public static double AlgaeHuman = 0.103; // using bore encoder
		//public static double AlgaeFloor = -10;
		public static double AlgaeFloor = 0.103; // using bore encoder

		public static double Start = 0.103; // using bore encoder

		//public static double P = 1.5;
		//public static double P = 0.05;
		//public static double P = 1.5; // using bore encoder
		public static double P = 2.0; // using bore encoder and maxmotion
		//public static double I = 0.0;
		public static double I = 0.01; // using bore encoder and maxmotion
		//public static double D = 0.0;
		//public static double D = 0.0;
		//public static double D = 10.0;
		public static double D = 0.0; // using bore encoder and maxmotion

		public static int motor_id = 20;
	}

	public static class EndEffectorConstants {
		public static double StoppedMotor1 = 0.0;
		// public static double StoppedMotor2 = 0.0;
		public static double IntakeHoldAlgaeMotor1 = -0.1;
		// public static double IntakeHoldAlgaeMotor2 = -0.1;
		public static double IntakeAlgaeFloorMotor1 = -0.15;
		// public static double IntakeAlgaeFloorMotor2 = -0.2;
		public static double IntakeCoralHumanElementMotor1 = .25;
		// public static double IntakeCoralHumanElementMotor2 = -0.25;
		public static double EjectAlgaeFloorMotor1 = 0.2;
		// public static double EjectAlgaeFloorMotor2 = 0.2;
		public static double EjectCoralMotor1 = 0.5;
		// public static double EjectCoralMotor2 = 0.0;
		public static double EjectCoralMotor1Slow = 0.5;
		// public static double EjectCoralMotor2Slow = 0.0;
		
		//public static double P = 1.5;
		public static double P = 1.0;
		public static double I = 0.0;
		//public static double D = 0.0;
		public static double D = 0.01;

		public static int motor_id = 19;
		// public static int motor2_id = 19;

		public static double OutputCurrentLimitMotor1 = 150.0;
		public static double OutputCurrentLimitMotor2 = 5.0;
		public static int kBeamBreakerPort = 0;
	}

	public static class ElevatorConstants {
		public static int motor_id = 33;
		public static int motor2_id = 32;
		//public static double P = 0.1; // bore encoder testing
		public static double P = 0.055;
		//public static double P = 5;
		public static double I = 0.00002;
		//public static double D = 0.02;
		public static double D = 0.002;

		public static int limitSwitch_id = 1;

		public static double Start = 0.0;
		//public static double Start = -0.4;  // Bore encoder testing
		public static double Stopped = 0.0;
		//public static double CoralHuman = 5.0;
		public static double CoralHuman = 0.0;
		public static double CoralL4 = 13.663;
		//public static double CoralL3 = 13.45;
		public static double CoralL3 = 0.000;
		public static double CoralL2 = 11.167;
		public static double CoralL1 = 6.5;
		public static double AlgaeHuman = 1.0;
		public static double AlgaeL3 = .595;
		public static double AlgaeL2 = .595;
		public static double AlgaeL1 = .595;
		public static double AlgaeShoot = 1.0;
		public static double AlgaeFloor = 2.0;
		public static double ClimberUp = 12.0;
		public static double ClimberDown = 9.0;
	}
}
