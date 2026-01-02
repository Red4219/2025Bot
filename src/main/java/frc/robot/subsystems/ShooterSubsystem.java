package frc.robot.subsystems;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.tools.Limelight;

public class ShooterSubsystem extends SubsystemBase {

    private double targetDistance = 0.0;
    private int targetId = 0;
    private boolean isSim = false;

    private SparkFlex shooterMotor = null;
    private SparkFlexConfig shooterConfig = null;
    private SparkFlexSim shooterFlexSim = null;
    
    private SparkMax turretMotor = null;
    private SparkMaxConfig turretConfig = null;
    private SparkMaxSim turretMotorSim = null;

    private SparkMax hoodMotor = null;
    private SparkMaxConfig hoodConfig = null;
    private SparkMaxSim hoodMotorSim = null;

    private double turretPosition = 0.0;
    private boolean hasTarget = false;

    private Limelight limelight = null;

    private double shooterP = Constants.ShooterConstants.shooterP;
    public GenericEntry entryShooterP = null;
    private double shooterI = Constants.ShooterConstants.shooterI;
    public GenericEntry entryShooterI = null;
    private double shooterD = Constants.ShooterConstants.shooterD;
    public GenericEntry entryShooterD = null;

    private double hoodP = Constants.ShooterConstants.shooterP;
    public GenericEntry entryHoodP = null;
    private double hoodI = Constants.ShooterConstants.shooterI;
    public GenericEntry entryHoodI = null;
    private double hoodD = Constants.ShooterConstants.shooterD;
    public GenericEntry entryHoodD = null;

    public ShooterSubsystem() {
        if(Constants.ShooterConstants.enabled) {
            isSim = RobotBase.isReal();

            // Shooter

            shooterMotor = new SparkFlex(Constants.ShooterConstants.shooterMotorId, MotorType.kBrushless);

            shooterConfig = new SparkFlexConfig();

            shooterConfig
                .inverted(Constants.ShooterConstants.invertShooterMotor)
                .idleMode(IdleMode.kCoast);

            shooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			    .pid(
				    Constants.ShooterConstants.shooterP,
				    Constants.ShooterConstants.shooterI,
				    Constants.ShooterConstants.shooterD
			    );
            
            shooterConfig.signals.primaryEncoderPositionPeriodMs(5);

            shooterConfig.apply(shooterConfig);

            // Turret

            turretMotor = new SparkMax(Constants.ShooterConstants.turretMotorId, MotorType.kBrushless);

            turretConfig = new SparkMaxConfig();

            turretConfig
                .inverted(Constants.ShooterConstants.invertTurretMotor)
                .idleMode(IdleMode.kBrake);

            turretConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			    .pid(
				    Constants.ShooterConstants.turretP,
				    Constants.ShooterConstants.turretI,
				    Constants.ShooterConstants.turretD
			    );

            turretConfig.signals.primaryEncoderPositionPeriodMs(5);

            turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Hood

            hoodMotor = new SparkMax(Constants.ShooterConstants.hoodMotorId, MotorType.kBrushless);

            hoodConfig = new SparkMaxConfig();

            hoodConfig
                .inverted(Constants.ShooterConstants.invertHoodMotor)
                .idleMode(IdleMode.kBrake);

            hoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			    .pid(
				    Constants.ShooterConstants.hoodP,
				    Constants.ShooterConstants.hoodI,
				    Constants.ShooterConstants.hoodD
			    );

            hoodConfig.signals.primaryEncoderPositionPeriodMs(5);

            hoodMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            if(isSim) {
                shooterFlexSim = new SparkFlexSim(shooterMotor, DCMotor.getNeoVortex(1));
                turretMotorSim = new SparkMaxSim(turretMotor, DCMotor.getNeo550(1));
                hoodMotorSim = new SparkMaxSim(hoodMotor, DCMotor.getNeo550(1));
            }

            // Limelight

            limelight = new Limelight("limelight");

            ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

            shooterTab.addBoolean("HasTarget", this::hasTarget);
            shooterTab.addDouble("Velocity", this::getFlyWheelVelocity);
            shooterTab.addDouble("Hood Position", this::getHoodPosition);

            // Shooter PID
            
            entryShooterP = shooterTab.add("ShooterP", shooterP)
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();

            entryShooterI = shooterTab.add("ShooterI", shooterI)
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();

            entryShooterD = shooterTab.add("ShooterD", shooterD)
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();
            
            // Hood PID

            entryHoodP = shooterTab.add("HoodP", shooterP)
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();

            entryHoodI = shooterTab.add("HoodI", shooterI)
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();

            entryHoodD = shooterTab.add("HoodD", shooterD)
            .withWidget(BuiltInWidgets.kTextView)
			.getEntry();
        }
    }

    public double getShooterP() {
        return shooterP;
    }

    public double getHoodPosition() {
        return hoodMotor.getAbsoluteEncoder().getPosition();
    }

    public double getFlyWheelVelocity() {
        return turretMotor.getAbsoluteEncoder().getVelocity();
    }

    public void setTargetId(int targetId) {
        this.targetId = targetId;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    @Override
	public void simulationPeriodic() {

	}

	@Override
	public void periodic() {
        if(Constants.ShooterConstants.enabled) {
            RawDetection detection = limelight.targetDetection();

            if(detection != null) {
                hasTarget = true;
            } else {
                hasTarget = false;
            }
        }

        if(Constants.ShooterConstants.debug) {

            // Shooter

            if(
				entryShooterP.getDouble(0.0) != shooterP
				|| entryShooterI.getDouble(0.0) != shooterI
				|| entryShooterD.getDouble(0.0) != shooterD
            ) {

                shooterP = entryShooterP.getDouble(0.0);
                shooterI = entryShooterI.getDouble(0.0);
                shooterD = entryShooterD.getDouble(0.0);

                shooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			    .pid(
				    shooterP,
				    shooterI,
				    shooterD
			    );

                shooterConfig.apply(shooterConfig);
            }

            // Hood

            if(
				entryHoodP.getDouble(0.0) != hoodP
				|| entryHoodI.getDouble(0.0) != hoodI
				|| entryHoodD.getDouble(0.0) != hoodD
            ) {

                hoodP = entryHoodP.getDouble(0.0);
                hoodI = entryHoodI.getDouble(0.0);
                hoodD = entryHoodD.getDouble(0.0);

                hoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			    .pid(
				    hoodP,
				    hoodI,
				    hoodD
			    );

            }
        }
    }
}
