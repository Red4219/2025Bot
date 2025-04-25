package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    public enum ArmState {
        Start,
        ClearCoral,
        CoralHuman,
        CoralL4,
        CoralL3,
        CoralL2,
        CoralL1,
        AlgaeHuman,
        AlgaeL3,
        AlgaeL2,
        AlgaeL1,
        AlgaeShoot,
        ArmFloor
    }

    private boolean isSim = false;
    private ArmState state = ArmState.Start;
    private double targetPosition = Constants.ArmConstants.Start;
    private SparkMax motor = null;
    private SparkMaxSim motorSim = null;
    //private SparkClosedLoopController pid = null;
    private SparkMaxConfig config = new SparkMaxConfig();

    // need to look into using this
    private ProfiledPIDController profiledPIDController = new ProfiledPIDController(
        0.1,
        0.0, 
        0.1, 
        new TrapezoidProfile.Constraints(2, 2),
        0.02
    ); 

    private double p = Constants.ArmConstants.P;
    private double i = Constants.ArmConstants.I;
    private double d = Constants.ArmConstants.D;

    // these values were calculate using https://www.reca.lc/arm
    //private ArmFeedforward armFeedforward = new ArmFeedforward(1.1, 5.02, 0.25);
    //private ArmFeedforward armFeedforward = new ArmFeedforward(0.1, 2.0, 0.25);

    public ArmSubsystem() {

        if (Constants.kEnableArm) {

            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            motor = new SparkMax(Constants.ArmConstants.motor_id, MotorType.kBrushless);

		    if(isSim) {
			    motorSim = new SparkMaxSim(motor, DCMotor.getNeoVortex(1));
		    }

            setConfig();

		    //pid = motor.getClosedLoopController();

            if (Constants.kEnableDebugArm) {

                Shuffleboard.getTab("Arm")
                    .addDouble("Position", this::getPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("Arm")
                    .addDouble("Target", this::getTargetPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                SmartDashboard.putData(this);
                Shuffleboard.getTab("Arm").add(this);

            }
        }
    }

    public void setDesiredState(ArmState state) {

        if(this.state == state) {
            // trying to set the state to the state we are already at
            // just returning to save cycles
            return;
        }

        switch(state) {
            case ClearCoral:
                targetPosition = Constants.ArmConstants.ClearCoral;
                break;
            case AlgaeHuman:
                targetPosition = Constants.ArmConstants.AlgaeHuman;
                break;
            case AlgaeL1:
                targetPosition = Constants.ArmConstants.AlgaeL1;
                break;
            case AlgaeL2:
                targetPosition = Constants.ArmConstants.AlgaeL2;
                break;
            case AlgaeL3:
                targetPosition = Constants.ArmConstants.AlgaeL3;
                break;
            case AlgaeShoot:
                targetPosition = Constants.ArmConstants.AlgaeL3;
                break;
            case ArmFloor:
                targetPosition = Constants.ArmConstants.AlgaeFloor;
                break;
            case CoralL1:
                targetPosition = Constants.ArmConstants.CoralL1;
                break;
            case CoralL2:
                targetPosition = Constants.ArmConstants.CoralL2;
                break;
            case CoralL3:
                targetPosition = Constants.ArmConstants.CoralL3;
                break;
            case CoralL4:
                targetPosition = Constants.ArmConstants.CoralL4;
                break;
            case CoralHuman:
                targetPosition = Constants.ArmConstants.CoralHuman;
                break;
            case Start:
                targetPosition = Constants.ArmConstants.Start;
                break;
            default:
                //targetPosition = 0.0;
                targetPosition = Constants.ArmConstants.Start;
                break;
            
        }

        this.state = state;
        //drivePID.setReference(targetPosition, ControlType.kPosition);
    }

    public ArmState getDesiredState() {
        return state;
    }

    @Override
	public void periodic() {

        if (Constants.kEnableArm) {
    
            motor.set(
                profiledPIDController.calculate(motor.getAbsoluteEncoder().getPosition(), targetPosition)
            );

            if (isSim) {
                motorSim.iterate(
                        // 0.1,
                        // desiredState.speedMetersPerSecond,
                        motor.getOutputCurrent(),
                        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                        0.02);
            }
        }

        
    }
    
    private void setConfig() {
        config = new SparkMaxConfig();

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(9)
            .velocityConversionFactor(9);
        config.closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
		    .pid(
			    p, 
                i, 
                d
			);

        config.absoluteEncoder.zeroOffset(0.5);

        // Set MAXMotion parameters
        config.closedLoop.maxMotion
            //.maxVelocity(6784)
            .maxVelocity(400)
            .maxAcceleration(100)
            //.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .allowedClosedLoopError(.05);

        config.signals.primaryEncoderPositionPeriodMs(5);

        motor.configure(
		    config, 
			ResetMode.kResetSafeParameters, 
			PersistMode.kPersistParameters
		);

        profiledPIDController.setPID(p, i, d);
        profiledPIDController.setTolerance(0.05);
    }

    public double getPosition() {

        if(isSim) {
            return motorSim.getRelativeEncoderSim().getPosition();
        }

        //return motor.getEncoder().getPosition();
        return  motor.getAbsoluteEncoder().getPosition();
    }

    public double getTargetPosition() {
        return this.targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public boolean atTargetPosition() {
        return Math.abs(getPosition() - targetPosition) < 0.05;
    }

    public double getP() {
        return this.p;
    }

    public void setP(double p) {
        this.p = p;
        setConfig();
    }

    public double getI() {
        return this.i;
    }

    public void setI(double i) {
        this.i = i;
        setConfig();
    }

    public double getD() {
        return this.d;
    }

    public void setD(double d) {
        this.d = d;
        setConfig();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("Target", this::getTargetPosition, this::setTargetPosition);
        builder.addDoubleProperty("Position", this::getPosition,null);
        builder.addBooleanProperty("At Target", this::atTargetPosition, null);
    }
}
