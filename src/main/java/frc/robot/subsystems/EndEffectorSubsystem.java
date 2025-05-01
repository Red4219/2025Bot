package frc.robot.subsystems;


import java.util.EnumSet;
import java.util.Map;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.mechanisms.LED.LEDStatus;
import frc.robot.subsystems.ArmSubsystem.ArmState;

public class EndEffectorSubsystem extends SubsystemBase {
    public enum EndEffectorState {
        Stopped,
        IntakeHoldAlgae,
        IntakeAlgaeFloor,
        IntakeCoralHumanElement,
        EjectAlgaeFloor,
        EjectCoralFront,
        EjectCoralBack,
        EjectCoralFrontNoCheck,
        EjectCoralBackNoCheck
    }

    private boolean isSim = false;
    private EndEffectorState state = EndEffectorState.Stopped;
    private double targetVelocity1 = 0.0;
    private double targetVelocity2 = 0.0;
    private SparkFlex motor = null;
    private SparkFlexSim motorSim = null;
    private SparkClosedLoopController pid = null;
    private SparkFlexConfig config = new SparkFlexConfig();
    private double p = Constants.EndEffectorConstants.P;
    private double i = Constants.EndEffectorConstants.I;
    private double d = Constants.EndEffectorConstants.D;

    // private SparkMax motor2 = null;
    // private SparkMaxSim motor2Sim = null;
    // private SparkClosedLoopController pid2 = null;
    private SparkMaxConfig config2 = new SparkMaxConfig();

    private boolean hasCoral = false;
    //private boolean hasAlgae = false;

    private DigitalInput beamBreaker = new DigitalInput(Constants.EndEffectorConstants.kBeamBreakerPort);

    public EndEffectorSubsystem() {
        if(Constants.kEnableEndEffector) {
            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            motor = new SparkFlex(Constants.EndEffectorConstants.motor_id, MotorType.kBrushless);
            // motor2 = new SparkMax(Constants.EndEffectorConstants.motor_id, MotorType.kBrushless);

            if(isSim) {
			    motorSim = new SparkFlexSim(motor, DCMotor.getNeoVortex(1));
                // motor2Sim = new SparkMaxSim(motor2, DCMotor.getNEO(1));
		    }

            setConfig();

		    pid = motor.getClosedLoopController();
            // pid2 = motor2.getClosedLoopController();

            if (Constants.kEnableDebugEndEffector) {

                Shuffleboard.getTab("End Effector")
                    .addDouble("Velocity1", this::getVelocity1)
                    .withWidget(BuiltInWidgets.kTextView);

                    // Shuffleboard.getTab("End Effector")
                    // .addDouble("Velocity2", this::getVelocity1)
                    // .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("End Effector")
                    .addDouble("Target Velocity 1", this::getTargetVelocity1)
                    .withWidget(BuiltInWidgets.kTextView);

                // Shuffleboard.getTab("End Effector")
                //     .addDouble("Target Velocity 2", this::getTargetVelocity2)
                //     .withWidget(BuiltInWidgets.kTextView);

                SmartDashboard.putData(this);
                Shuffleboard.getTab("End Effector").add(this);

            }   
        }
    }

    public void setDesiredState(EndEffectorState state) {

        if(
            state == EndEffectorState.IntakeAlgaeFloor
            && this.state == EndEffectorState.IntakeAlgaeFloor
        ) {
            // the IntakeAlgaeFloor has been pressed twice so
            // stop the motors
            state = EndEffectorState.Stopped;
        } else if(
            state == EndEffectorState.EjectAlgaeFloor
            && this.state == EndEffectorState.EjectAlgaeFloor
        ) {
            // the EjectAlgaeFloor has been pressed twice so
            // stop the motors
            state = EndEffectorState.Stopped;
        } else if(
            state == EndEffectorState.EjectCoralFront
            && this.state == EndEffectorState.EjectCoralFront
        ) {
            // The EjectCoralFront was sent twice so stop it
            state = EndEffectorState.Stopped;
        } else if(
            state == EndEffectorState.EjectCoralBack
            && this.state == EndEffectorState.EjectCoralBack
        ) {
            // The EjectCoralBack was sent twice so stop it
            state = EndEffectorState.Stopped;
        } else if(
            state == EndEffectorState.EjectCoralFrontNoCheck
            && this.state == EndEffectorState.EjectCoralFrontNoCheck
        ) {
            // The EjectCoralFront was sent twice so stop it
            state = EndEffectorState.Stopped;
        } else if(
            state == EndEffectorState.EjectCoralBackNoCheck
            && this.state == EndEffectorState.EjectCoralBackNoCheck
        ) {
            // The EjectCoralFront was sent twice so stop it
            state = EndEffectorState.Stopped;
        } else if(
            state == EndEffectorState.IntakeHoldAlgae
            && this.state == EndEffectorState.IntakeHoldAlgae
        ) {
            // The EjectCoralFront was sent twice so stop it
            state = EndEffectorState.Stopped;
        } else if (this.state == state) {
            // trying to set the state to the state we are already at
            // just returning to save cycles
            return;
        } else{
            this.state = state;
        }

        switch (state) {
            case Stopped:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - Stopped"));
                targetVelocity1 = 0.0;
                // targetVelocity2 = 0.0;
                break;
            case IntakeAlgaeFloor:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - IntakeAlgaeFloor"));
                targetVelocity1 = Constants.EndEffectorConstants.IntakeAlgaeFloorMotor1;
                // targetVelocity2 = Constants.EndEffectorConstants.IntakeAlgaeFloorMotor2;
                break;
            case IntakeCoralHumanElement:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - IntakeCoralHumanElement"));
                targetVelocity1 = -Constants.EndEffectorConstants.IntakeCoralHumanElementMotor1;
                // targetVelocity2 = -Constants.EndEffectorConstants.IntakeCoralHumanElementMotor2;
                break;
            case EjectAlgaeFloor:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - EjectAlgaeFloor"));
                targetVelocity1 = Constants.EndEffectorConstants.EjectAlgaeFloorMotor1;
                // targetVelocity2 = Constants.EndEffectorConstants.EjectAlgaeFloorMotor2;
                break;
            case EjectCoralFront:

                System.out.println(("EndEffectorSubsystem::setDesiredState() - EjectCoralFront"));

                if(RobotContainer.armSubsystem.getDesiredState() == ArmState.CoralL1) {
                    // If we are trying to eject out for a Coral L1, slow it down
                    targetVelocity1 = Constants.EndEffectorConstants.EjectCoralMotor1;
                    // targetVelocity2 = Constants.EndEffectorConstants.EjectCoralMotor2;
                } else {
                    // we are not trying to eject to Coral L1 so go the requested speed
                    targetVelocity1 = Constants.EndEffectorConstants.EjectCoralMotor1;
                    // targetVelocity2 = Constants.EndEffectorConstants.EjectCoralMotor2;
                }
                //targetVelocity1 = Constants.EndEffectorConstants.EjectCoralMotor1;
                //targetVelocity2 = Constants.EndEffectorConstants.EjectCoralMotor2;
                break;
            case EjectCoralBack:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - EjectCoralBack"));
                targetVelocity1 = Constants.EndEffectorConstants.EjectCoralMotor1;
                // targetVelocity2 = -Constants.EndEffectorConstants.EjectCoralMotor2;
                break;
            case EjectCoralFrontNoCheck:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - EjectCoralFrontNoCheck"));
                targetVelocity1 = Constants.EndEffectorConstants.EjectCoralMotor1;
                // targetVelocity2 = Constants.EndEffectorConstants.EjectCoralMotor2;
                break;
            case EjectCoralBackNoCheck:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - EjectCoralBackNoCheck"));
                targetVelocity1 = Constants.EndEffectorConstants.EjectCoralMotor1;
                // targetVelocity2 = -Constants.EndEffectorConstants.EjectCoralMotor2;
                break;
            case IntakeHoldAlgae:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - IntakeHoldAlgae"));
                targetVelocity1 = Constants.EndEffectorConstants.IntakeHoldAlgaeMotor1;
                // targetVelocity2 = Constants.EndEffectorConstants.IntakeHoldAlgaeMotor2;
                break;
            default:
                System.out.println(("EndEffectorSubsystem::setDesiredState() - default"));
                targetVelocity1 = 0.0;
                // targetVelocity2 = 0.0;
                break;
        }

        this.state = state;
    }

    @Override
	public void periodic() {

        if (Constants.kEnableEndEffector) {
            if (motor1OutputCurrent() >= 60 && this.state == EndEffectorState.IntakeCoralHumanElement) {
                setDesiredState(EndEffectorState.Stopped);
                System.out.println("EndEffectorSubsystem::periodic() - Stopped");
            }
            // System.out.println(motor1OutputCurrent());   

            // if(!beamBreaker.get()) {
            //     hasCoral = true;
            // } else {
            //     hasCoral = false;
            // }

            //if(!hasCoral && !hasAlgae) {
            if(true) {
                // We do not have the coral or algae
                RobotContainer.led1.setStatus(LEDStatus.ready);
            } else if(hasCoral && RobotContainer.led1.getStatus() == LEDStatus.targetAquired) { 
                RobotContainer.led1.setStatus(LEDStatus.targetAquired);
            } else if(hasCoral) {
                // we have the coral
                RobotContainer.led1.setStatus(LEDStatus.hasCoral);
            } /*else if(hasAlgae) {
                // we have the algae
                RobotContainer.led1.setStatus(LEDStatus.hasAlgae);
            }*/

            switch (state) {
                case IntakeCoralHumanElement:
                        // we are trying to intake the coral and the beam breaker says we have it so
                        // stop the motors
                        
                        // motor2.set(0.0);
                    
                        // We don't have the coral so run the intake motors
                    motor.set(targetVelocity1);
                        // motor2.set(targetVelocity2);
                    
                    break;
                case IntakeAlgaeFloor:                    
                    // Run the motors
                    motor.set(Constants.EndEffectorConstants.IntakeAlgaeFloorMotor1);
                    // motor2.set(Constants.EndEffectorConstants.IntakeAlgaeFloorMotor2);

                    //motor.set(Constants.EndEffectorConstants.IntakeAlgaeFloorMotor1);
                    break;
                case EjectAlgaeFloor:
                    motor.set(Constants.EndEffectorConstants.EjectAlgaeFloorMotor1);
                    // motor2.set(Constants.EndEffecgtorConstants.EjectAlgaeFloorMotor2);
                    //hasAlgae = false;
                    break;
                case EjectCoralFront:
                    //if(hasCoral) {
                        // we have the coral so eject it
                        // motor2.set(targetVelocity1);
                        motor.set(targetVelocity1);
                    /* } else {
                        // we don't have the coral so stop the motors
                        motor.set(0.0);
                        motor2.set(0.0);
                    }*/
                    break;
                case EjectCoralBack:
                    //if(hasCoral) {
                        //System.out.println("ejecting the back");
                        // we have the coral so eject it
                        // motor2.set(targetVelocity1);
                        motor.set(targetVelocity1);
                    /* } else {
                        // we don't have the coral so stop the motors
                        motor.set(0.0);
                        motor2.set(0.0);
                    }*/
                    break;
                case EjectCoralFrontNoCheck:
                    // motor2.set(targetVelocity1);
                    motor.set(targetVelocity1);
                    break;
                case EjectCoralBackNoCheck:
                    // motor2.set(targetVelocity1);
                    motor.set(targetVelocity1);
                    break;
                case Stopped:
                    // stop the motors
                    motor.set(0.0);
                    // motor2.set(0.0);
                default:
                    motor.set(targetVelocity1);
                    //motor2.set(targetVelocity2);
                    break;
            }
        }

        if(isSim) {
            this.motorSim.iterate(
				targetVelocity1,
        		RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        		0.02
			);

            // this.motor2Sim.iterate(
			// 	targetVelocity2,
        	// 	RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        	// 	0.02
			// );
        }
    }

    private void setConfig() {
        // Vortex
        config = new SparkFlexConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		    .pid(
			    p, 
                i, 
                d
			);
        config.signals.primaryEncoderPositionPeriodMs(5);

        motor.configure(
		    config, 
			ResetMode.kResetSafeParameters, 
			PersistMode.kPersistParameters
		);

        // Neo 550
        config2 = new SparkMaxConfig();

        config2.inverted(true)
            .idleMode(IdleMode.kBrake);
        config2.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        config2.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
			    p, 
                i, 
                d
			);

        config2.signals.primaryEncoderPositionPeriodMs(5);
    }

    public boolean hasCoral() {
        return false;
    }

    public double getVelocity1() {

        if(isSim) {
            return motorSim.getRelativeEncoderSim().getVelocity();
        }

        return motor.getEncoder().getVelocity();
    }

    // public double getVelocity2() {

    //     if(isSim) {
    //         // return motor2Sim.getRelativeEncoderSim().getVelocity();
    //     }

    //     // return motor2.getEncoder().getVelocity();
    // }

    public double getTargetVelocity1() {

        return this.targetVelocity1;
    }

    public void setTargetVelocity1(double targetVelocity) {
        this.targetVelocity1 = targetVelocity;
    }

    public double getTargetVelocity2() {

        return this.targetVelocity1;
    }

    public void setTargetVelocity2(double targetVelocity) {
        this.targetVelocity2 = targetVelocity;
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

    public double motor1OutputCurrent() {
        return motor.getOutputCurrent();
    }

    // public double motor2OutputCurrent() {
    //     return motor2.getOutputCurrent();
    // }

    public EndEffectorState getState() {
        return this.state;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("Target Velocity1", this::getTargetVelocity1, this::setTargetVelocity1);
        builder.addDoubleProperty("Target Velocity2", this::getTargetVelocity2, this::setTargetVelocity2);
        builder.addDoubleProperty("Velocity1", this::getVelocity1, null);
        // builder.addDoubleProperty("Velocity2", this::getVelocity2, null);
        builder.addBooleanProperty("Has Coral", this::hasCoral, null);
        builder.addDoubleProperty("Motor1 OutputCurrent", this::motor1OutputCurrent, null);
        // builder.addDoubleProperty("Motor2 OutputCurrent", this::motor2OutputCurrent, null);
    }
}
