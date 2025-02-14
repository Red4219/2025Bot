package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.Map;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorState {
        Start,
        CoralHuman,
        CoralL4,
        CoralL3,
        CoralL2,
        CoralL1,
        AlgaeHuman,
        AlgaeL3,
        AlgaeL2,
        AlgaeL1,
        AlgaeShoot
    }

    private boolean isSim = false;
    private ElevatorState state = ElevatorState.Start;
    private double targetPosition = 0.0;
    private SparkMax motor = null;
    private SparkMaxSim motorSim = null;
    private SparkMax motor2 = null;
    private SparkMaxSim motor2Sim = null;
    private SparkClosedLoopController pid = null;
    private SparkMaxConfig config = new SparkMaxConfig();
    private double p = Constants.ElevatorConstants.P;
    private double i = Constants.ElevatorConstants.I;
    private double d = Constants.ElevatorConstants.D;

    private double currentPosition = 0.0;
    private double previousValue = 0.0;
    private int revolutionCount = 0;

    public ElevatorSubsystem() {

        if(Constants.kEnableElevator) {

            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            motor = new SparkMax(Constants.ElevatorConstants.motor_id, MotorType.kBrushless);
            motor2 = new SparkMax(Constants.ElevatorConstants.motor2_id, MotorType.kBrushless);

            if(isSim) {
                motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
                motor2Sim = new SparkMaxSim(motor2, DCMotor.getNEO(1));
            }

            setConfig();

            pid = motor.getClosedLoopController();

            if (Constants.kEnableDebugElevator) {

                Shuffleboard.getTab("Elevator")
                    .addDouble("Position", this::getPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("Elevator")
                    .addDouble("Target", this::getPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                SmartDashboard.putData(this);
                Shuffleboard.getTab("Elevator").add(this);
            }
        }
    }

    public void setDesiredState(ElevatorState state) {

        // Are we sending the same state again?  If so act like a toggle and stop
        //if (this.state == state) {
        //    targetPosition = Constants.SliderConstants.Stopped;
        //} else {

            switch (state) {
                case Start:
                    targetPosition = Constants.ElevatorConstants.Start;
                    break;
                case CoralHuman:
                    targetPosition = Constants.ElevatorConstants.CoralHuman;
                    break;
                case CoralL4:
                    targetPosition = Constants.ElevatorConstants.CoralL4;
                    break;
                case CoralL3:
                    targetPosition = Constants.ElevatorConstants.CoralL3;
                    break;
                case CoralL2:
                    targetPosition = Constants.ElevatorConstants.CoralL2;
                    break;
                case CoralL1:
                    targetPosition = Constants.ElevatorConstants.CoralL1;
                    break;
                case AlgaeL3:
                    targetPosition = Constants.ElevatorConstants.AlgaeL3;
                    break;
                case AlgaeL2:
                    targetPosition = Constants.ElevatorConstants.AlgaeL2;
                    break;
                case AlgaeL1:
                    targetPosition = Constants.ElevatorConstants.AlgaeL1;
                    break;
                case AlgaeShoot:
                    targetPosition = Constants.ElevatorConstants.AlgaeShoot;
                    break;
                case AlgaeHuman:
                    targetPosition = Constants.ElevatorConstants.AlgaeHuman;
                    break;
                default:
                    targetPosition = 0.0;
                    break;
            }
        //}

        this.state = state;
    }

    public ElevatorState getState() {
        return state;
    }

    @Override
	public void periodic() {

        if (Constants.kEnableElevator) {
            pid.setReference(targetPosition, ControlType.kPosition);
            //pid.setReference(targetPosition, SparkBase.ControlType.kMAXMotionPositionControl);

            if (isSim) {
                motorSim.iterate(
                        // 0.1,
                        // desiredState.speedMetersPerSecond,
                        motor.getOutputCurrent(),
                        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                        0.02
                    );
            }
        }
    }

    private void setConfig() {
        // Vortex
        config = new SparkMaxConfig();

        config
            .inverted(true)
            //.smartCurrentLimit(200)
            .idleMode(IdleMode.kCoast);
        //config.encoder
            //.positionConversionFactor(25)
            //.velocityConversionFactor(25);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            //.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            //.feedbackSensor(FeedbackSensor.kAnalogSensor)
            //.positionWrappingEnabled(true)
		    .pid(
			    p, 
                i, 
                d
			).outputRange(-1, 1);

        // Set MAXMotion parameters
        //config.closedLoop.maxMotion
            //.maxVelocity(6784)
            //.maxVelocity(1)
            //.maxAcceleration(1)
            //.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            //.allowedClosedLoopError(.1);

        config.signals.primaryEncoderPositionPeriodMs(5);
        config.signals.primaryEncoderPositionAlwaysOn(true);

        motor.configure(
		    config, 
			ResetMode.kResetSafeParameters, 
			PersistMode.kPersistParameters
		);

        SparkMaxConfig config2 = new SparkMaxConfig();

        config2
            .idleMode(IdleMode.kCoast);
        config2.follow(Constants.ElevatorConstants.motor_id);

        motor2.configure(
            config2, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    public double getPosition() {

        if(isSim) {
            return motorSim.getRelativeEncoderSim().getPosition();
        }

        return motor.getEncoder().getPosition();
        //currentPosition = motor.getEncoder().getPosition();
        /*currentPosition = motor.getAbsoluteEncoder().getPosition();
        //currentPosition = motor.getAlternateEncoder().getPosition();

        if(currentPosition < .2 && previousValue > .8) {
            revolutionCount--;
        } else if(currentPosition > .8 && previousValue < .2) {
            revolutionCount++;
        }

        previousValue = currentPosition;


        //previousValue = revolutionCount + motor.getAbsoluteEncoder().getPosition();
        //return revolutionCount + position;

        return currentPosition;*/
    }

    public double getTargetPosition() {

        return this.targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
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

    public int getRevolutions() {
        return this.revolutionCount;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        //builder.setActuator(true);
        //builder.setSafeState(this::disable);
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("Target", this::getTargetPosition, this::setTargetPosition);
        builder.addDoubleProperty("Position", this::getPosition,null);
        builder.addDoubleProperty("Revolutions", this::getRevolutions,null);
    }
}
