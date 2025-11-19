// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.tools.parts.PIDGains;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.sim.TalonFXSimState;



public class SwerveModule {
	/** Creates a new SwerveModule. */
	//private final CANBus kCANBus = new CANBus();

	// private final SparkFlex driveMotor;
	private final TalonFX driveMotor;
	private VelocityDutyCycle targetVelo = new VelocityDutyCycle(0);
	// private SparkFlexSim driveFlexSim = null;
	private TalonFXSimState talonFXSimState = null;
	private final SparkMax turningMotor;
	private SparkMaxSim turningMaxSim = null;
	private final CANcoder cancoder;
	// private final RelativeEncoder driveEncoder;
	

	// private final SparkClosedLoopController sparkDrivePID;
	private final ProfiledPIDController m_turningPIDController;

	public final double angleZero;

	private final String moduleName;
	private Rotation2d _simulatedAbsoluteEncoderRotation2d = null;

	private double m_moduleAngleRadians;
	private Rotation2d m_moduleAngleRotation2d = new Rotation2d();
	//private SwerveModuleState optimizedState;
	private double angularPIDOutput;
	private double angularFFOutput;
	private double turnOutput;
	private boolean isSim = false;
	private ShuffleboardTab swerveTab = null;
	private Orchestra kOrchestra = null;
	//private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

	SparkMaxConfig turnConfig = null;
	SparkFlexConfig driveConfig = null;
	
	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);

	private DCMotorSim m_motorSimModel = null;

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPID,
			PIDGains drivePID,
			boolean invertTurningMotor,
			// boolean invertDriveMotor
			InvertedValue invertDriveMotor
			) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		if(RobotBase.isReal()) {
			isSim = false;
		} else {
			isSim = true;
			_simulatedAbsoluteEncoderRotation2d = new Rotation2d(0.0);
		}

		// Initialize the motors
		// driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
		driveMotor = new TalonFX(driveMotorChannel);
		
		/*if (ModuleConstants.enableMusic){
			kOrchestra = new Orchestra();
			kOrchestra.addInstrument(driveMotor);
		}*/


		if(isSim) {
			// driveFlexSim = new SparkFlexSim(driveMotor, DCMotor.getNeoVortex(1));
			talonFXSimState = driveMotor.getSimState();

			// This is for the siumulator to simulate the movement of the motors
			m_motorSimModel = new DCMotorSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(1),
					0.001, 
					Constants.ModuleConstants.kdriveGearRatioL3
				), 
				DCMotor.getKrakenX60Foc(1)
				,0.00, 0.00 // not sure about these
			);

			talonFXSimState.setSupplyVoltage(12.0);
		}
		
		turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

		if(isSim) {
			turningMaxSim = new SparkMaxSim(turningMotor, DCMotor.getNEO(1));
		}

		cancoder = new CANcoder(absoluteEncoderPort, Constants.kCanivoreCANBusName);
		cancoder.clearStickyFaults();
		
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();
		driveConfig
		.withMotorOutput(
			new MotorOutputConfigs()
				.withInverted(invertDriveMotor)
				.withNeutralMode(NeutralModeValue.Coast)
		);
		driveConfig.Audio.AllowMusicDurDisable = true;
		
		driveMotor.getConfigurator().apply(driveConfig);

		var slot0Configs = new Slot0Configs();
		slot0Configs.kS = 0.1;
		slot0Configs.kV = 0.12;
		slot0Configs.kP = drivePID.kP; // An error of 1 rps results in 0.11 V output
		slot0Configs.kI = drivePID.kI; // no output for integrated error
		slot0Configs.kD = drivePID.kD; // no output for error derivative
		driveMotor.getConfigurator().apply(slot0Configs);

		FeedbackConfigs feedbackConfig = new FeedbackConfigs();
		//ModuleConstants.kdriveGearRatioL3 * ModuleConstants.kwheelCircumference
		//feedbackConfig.withSensorToMechanismRatio(ModuleConstants.kdriveGearRatioL3 * ModuleConstants.kwheelCircumference);
		feedbackConfig.withSensorToMechanismRatio(ModuleConstants.kdriveGearRatioL3 );
		feedbackConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
		//feedbackConfig.withRotorToSensorRatio(ModuleConstants.kdriveGearRatioL3 * ModuleConstants.kwheelCircumference); 
		driveMotor.getConfigurator().apply(feedbackConfig);

		// driveConfig = new SparkFlexConfig();

		// driveConfig
        //     .inverted(invertDriveMotor)
        //     .idleMode(IdleMode.kCoast);

        // driveConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		// 	.pid(
		// 		drivePID.kP,
		// 		drivePID.kI,
		// 		drivePID.kD
		// 	);

        // driveConfig.signals.primaryEncoderPositionPeriodMs(5);

		// // Set the gear ratio
		// driveConfig.encoder
		// .positionConversionFactor(
		// 	ModuleConstants.kdriveGearRatioL3 * ModuleConstants.kwheelCircumference
		// 	//Divide the circumference by the gear ratio to get your conversion calculation
		// 	//ModuleConstants.kwheelCircumference / ModuleConstants.kdriveGearRatioL3 
		// )
		// .velocityConversionFactor(
		// 	ModuleConstants.kdriveGearRatioL3
		// 	* ModuleConstants.kwheelCircumference
		// 	//ModuleConstants.kwheelCircumference / ModuleConstants.kdriveGearRatioL3 
		// 	* (1d / 60d)
		// );

        // driveMotor.configure(
		// 	driveConfig, 
		// 	ResetMode.kResetSafeParameters, 
		// 	PersistMode.kPersistParameters
		// );

		// this.sparkDrivePID = driveMotor.getClosedLoopController();

		// driveEncoder = driveMotor.getEncoder();

		turnConfig = new SparkMaxConfig();

		turnConfig
            .inverted(invertTurningMotor)
            .idleMode(IdleMode.kCoast);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(
				Constants.ModuleConstants.kModuleDriveGains.kP, 
				Constants.ModuleConstants.kModuleDriveGains.kI, 
				Constants.ModuleConstants.kModuleDriveGains.kD
			);
        turnConfig.signals.primaryEncoderPositionPeriodMs(5);

        turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_turningPIDController = new ProfiledPIDController(
			angularPID.kP,
			angularPID.kI,
			angularPID.kD,
			new TrapezoidProfile.Constraints( // radians/s?
					2 * Math.PI * 600, // theoretical is 5676 RPM -> 94*2pi
					2 * Math.PI * 1200));

		m_turningPIDController.enableContinuousInput(0, Math.toRadians(360));

		if(Constants.kDebugDriveTrain == true) {
			// Swerve tab stuff
			swerveTab = Shuffleboard.getTab("Swerve");
			swerveTab.addDouble(moduleName + " Absolute", this::getAbsoluteHeading);
			swerveTab.addDouble(moduleName + " Meters", this::getDistanceMeters);
			swerveTab.addString(moduleName + " Abs. Status", this::getStatus);
		}
	}

	// TODO: cleanup methods make sure they are compatible with new krakens

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return (cancoder.getAbsolutePosition().refresh().getValueAsDouble() * 360);
	}

	public double getDistanceMeters() {
		// return driveEncoder.getPosition();
		//return (driveMotor.getRotorPosition().getValueAsDouble()*ModuleConstants.kdriveGearRatioL3*ModuleConstants.kwheelCircumference);
		return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kwheelCircumference;
		//return (driveMotor.getRotorPosition().getValueAsDouble());
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		m_moduleAngleRadians = Math.toRadians(cancoder.getAbsolutePosition(true).getValueAsDouble() * 360.0);

		if(isSim){
			m_moduleAngleRotation2d = _simulatedAbsoluteEncoderRotation2d;
			return new SwerveModulePosition(getDistanceMeters(), _simulatedAbsoluteEncoderRotation2d);
		}

		m_moduleAngleRotation2d = Rotation2d.fromDegrees(cancoder.getAbsolutePosition(true).getValueAsDouble() * 360.0);

		return new SwerveModulePosition(getDistanceMeters(), m_moduleAngleRotation2d);
	}

	// Sets the position of the swerve module
	public void setDesiredState(SwerveModuleState desiredState) {	

		if(isSim) {
			m_moduleAngleRadians = Math.toRadians(desiredState.angle.getDegrees());
			_simulatedAbsoluteEncoderRotation2d = desiredState.angle;
		}

		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		desiredState.optimize(m_moduleAngleRotation2d);

		angularPIDOutput = m_turningPIDController.calculate(m_moduleAngleRadians,
			desiredState.angle.getRadians());

		angularFFOutput = turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

		turnOutput = angularPIDOutput + angularFFOutput;

		turningMotor.setVoltage(turnOutput);
		
		if(isSim) {
			// sparkDrivePID.setReference(
			// 	desiredState.speedMetersPerSecond,
			// 	ControlType.kVelocity
			// );

			// driveFlexSim.iterate(
			// 	desiredState.speedMetersPerSecond,
        	// 	RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        	// 	0.02
			// );
			//talonFXSimState.setSupplyVoltage(desiredState.speedMetersPerSecond);
			//var motorVoltage = talonFXSimState.getMotorVoltageMeasure();
			//m_motorSimModel.setInput(desiredState.);
			//m_motorSimModel.setInputVoltage(desiredState.speedMetersPerSecond);
			//m_motorSimModel.setInputVoltage(velocityToRPS(desiredState.speedMetersPerSecond));
			//m_motorSimModel.setAngularVelocity(desiredState.speedMetersPerSecond);
			
			//m_motorSimModel.setAngularVelocity(metersPerSecondToRevolutionsPerSecond(desiredState.speedMetersPerSecond, 2));
			m_motorSimModel.setAngularVelocity(desiredState.speedMetersPerSecond);

			//m_motorSimModel.setInputVoltage(desiredState.speedMetersPerSecond);
			m_motorSimModel.update(0.020); // assumeds 20 ms loop time

			//talonFXSimState.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(Constants.ModuleConstants.kdriveGearRatioL3));
			talonFXSimState.setRawRotorPosition(m_motorSimModel.getAngularPosition());
			//talonFXSimState.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(Constants.ModuleConstants.kdriveGearRatioL3));
			talonFXSimState.setRotorVelocity(m_motorSimModel.getAngularVelocity());

		} else {
			// sparkDrivePID.setReference(
			// 	desiredState.speedMetersPerSecond,
			// 	ControlType.kVelocity
			// 	//ControlType.kMAXMotionVelocityControl
			// );
			//targetVelo.Velocity = velocityToRPS(desiredState.speedMetersPerSecond);
			targetVelo.Velocity = desiredState.speedMetersPerSecond;
			driveMotor.setControl(targetVelo);
		}

		if(Constants.kEnableDriveSubSystemLogger) {
			Logger.recordOutput("Motors/DriveMotorCurrentOutput_" + moduleName, driveMotor.getStatorCurrent().getValueAsDouble());
			Logger.recordOutput("Motors/DriveMotorTemp_" + moduleName, driveMotor.getDeviceTemp().getValueAsDouble());
			Logger.recordOutput("Motors/TurnMotorCurrentOutput_" + moduleName, turningMotor.getOutputCurrent());
			Logger.recordOutput("Motors/TurnMotorTemp_" + moduleName, turningMotor.getMotorTemperature());
		}
	}

	private double velocityToRPS(double velocity){
		// 	ModuleConstants.kdriveGearRatioL3
		// 	* ModuleConstants.kwheelCircumference
		// 	//ModuleConstants.kwheelCircumference / ModuleConstants.kdriveGearRatioL3 
		// 	* (1d / 60d)
		double veloRps = ((velocity/ModuleConstants.kwheelCircumference)*6.12)/60;
		return veloRps;

	}

	public static double linearVelocityToRevolutionsPerSecond(double linearVelocity, double radius) {
        if (radius <= 0) {
            throw new IllegalArgumentException("Radius must be a positive value.");
        }

        // 1. Calculate angular velocity in radians per second
        // ω = v / r
        //double angularVelocityRadPerSec = linearVelocity / radius;
		return linearVelocity / radius;

        // 2. Convert radians per second to revolutions per second
        // 1 revolution = 2π radians
        //double revolutionsPerSecond = angularVelocityRadPerSec / (2 * Math.PI);

        //return revolutionsPerSecond;
    }

	public static double metersPerSecondToRevolutionsPerSecond(double metersPerSecond, double radius) {
        if (radius <= 0) {
            throw new IllegalArgumentException("Radius must be a positive value.");
        }
        // Circumference of the circle
        double circumference = 2 * Math.PI * radius;

        // Revolutions per second = (meters per second) / (circumference per revolution)
        return metersPerSecond / circumference;
    }

	public void resetEncoders() {
		
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}

	double getAngleZero() {
		return this.angleZero;
	}

	String getStatus() {
		return cancoder.getMagnetHealth().getValue().name();
	}

	public void setTurningPID(double p, double i, double d) {
		m_turningPIDController.setPID(p, i, d);
	}

	public void simulatePeriodic() {
		
	}

	public void setDrivePID(double p, double i, double d) {
		driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(
				p,
				i,
				d
			);
	}
	public void selectSong(int song){
		kOrchestra.loadMusic(ModuleConstants.song[song]);
	}
	public void playSong(){
		kOrchestra.play();
	}
	public void pauseSong(){
		kOrchestra.pause();
	}
	public void stopSong(){
		kOrchestra.stop();
	}
}
