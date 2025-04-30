// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  
  public enum ClimberState {
    STOPPED,
    EXTEND,
    RETRACT
  }

  private double climberExtendSpeed;
  private double climberRetractSpeed;

  public ClimberState state = ClimberState.STOPPED;

  private double targetAngle = ClimbConstants.servoClosed; 
  private double p = Constants.ClimbConstants.P;
  private double i = Constants.ClimbConstants.I;
  private double d = Constants.ClimbConstants.D;

  private Servo servo = new Servo(Constants.ClimbConstants.servo_id);
  private SparkMax motor = new SparkMax(Constants.ClimbConstants.motor_id, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private SparkClosedLoopController pid = null;
  private double targetVelocity = 0.0;

  public ClimbSubsystem() {
    climberExtendSpeed = Constants.ClimbConstants.climberExtendSpeed;
    climberRetractSpeed = Constants.ClimbConstants.climberRetractSpeed;
    setConfig();

    if (Constants.kEnableDebugEndEffector) {

                Shuffleboard.getTab("Climber")
                    .addDouble("Position", this::getPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                    // Shuffleboard.getTab("End Effector")
                    // .addDouble("Velocity2", this::getVelocity1)
                    // .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("Climber")
                    .addDouble("Target Velocity", this::getTargetVelocity)
                    .withWidget(BuiltInWidgets.kTextView);

                // Shuffleboard.getTab("End Effector")
                //     .addDouble("Target Velocity 2", this::getTargetVelocity2)
                //     .withWidget(BuiltInWidgets.kTextView);

                SmartDashboard.putData(this);
                Shuffleboard.getTab("Climber").add(this);

            }  

  }

  public void setConfig() {
    

        config
            //.inverted(false) // bore encoder testing
            .inverted(true)
            //.smartCurrentLimit(200)
            .idleMode(IdleMode.kBrake);
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
			);//.outputRange(-1, 1);

        // Set MAXMotion parameters
        config.closedLoop.maxMotion
            //.maxVelocity(6784)
            .maxVelocity(100)
            .maxAcceleration(50)
            //.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .allowedClosedLoopError(.05);

        config.signals.primaryEncoderPositionPeriodMs(5);
        config.signals.primaryEncoderPositionAlwaysOn(true);

        motor.configure(
		    config, 
			ResetMode.kResetSafeParameters, 
			PersistMode.kPersistParameters
		);

    pid = motor.getClosedLoopController();

  }
  public void setState(ClimberState newState) {
    this.state = newState;

    switch (this.state) {
      case STOPPED:
        targetVelocity = 0.0;
        targetAngle = ClimbConstants.servoClosed;
        break;
      case EXTEND:
        targetVelocity = climberExtendSpeed;
        targetAngle = ClimbConstants.servoOpen;
        break;
      case RETRACT:
        targetVelocity = -climberRetractSpeed;
        targetAngle = ClimbConstants.servoOpen;
        break;
    }
  }
  
  @SuppressWarnings("unused")
  private double getPosition(){
    return motor.getEncoder().getPosition();
  }

  // private double getP(){
  //   return p;
  // }
  // private double getI(){
  //   return i;
  // }
  // private double getD(){
  //   return d;
  // }
  // private void setP(double p){
  //   this.p = p; 
  //   setConfig();
  // }
  // private void setI(double i){
  //   this.i = i; 
  //   setConfig();
  // }
  // private void setD(double d){
  //   this.d = d; 
  //   setConfig();
  // }

  private double getTargetVelocity(){
    return targetVelocity;
  }
  public void setTargetVelocity(double targetVelocity){
    this.targetVelocity = targetVelocity; 
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */


  @Override
  public void periodic() {
    motor.set(targetVelocity);
    servo.setAngle(targetAngle);
  }


  @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        //builder.setActuator(true);
        //builder.setSafeState(this::disable);
        // builder.addDoubleProperty("D", this::getD, this::setD);
        // builder.addDoubleProperty("I", this::getI, this::setI);
        // builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("Target Velocity", this::getTargetVelocity, this::setTargetVelocity);
        builder.addDoubleProperty("Position", this::getPosition,null);
        
    }
}
