package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import com.pathplanner.lib.pathfinding.Pathfinding;
//import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// This is needed for AdvantageScope
public class Robot extends LoggedRobot {
	private Command autonomousCommand;
	private ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
	private ElevatorSubsystem elevatorSystem = RobotContainer.elevatorSubsystem;
	private RobotContainer robotContainer;
	private ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
	//private REVPhysicsSim simulator;

	private NetworkTableInstance inst;
	private NetworkTable table;
	private IntegerSubscriber gotoPositionSubscriber;
	private IntegerPublisher gotoPositiionPublisher;

	@Override
	public void robotInit() {

		Pathfinding.setPathfinder(new LocalADStarAK());

		// This is for advantagekit
		Logger.addDataReceiver(new NT4Publisher());

		//PathPlannerServer.startServer(5811);

		robotContainer = new RobotContainer(!Robot.isReal());
		DriverStation.silenceJoystickConnectionWarning(true);

		if(Constants.kEnableLimelight) {
			for (int port = 5800; port <= 5809; port++) {
            	PortForwarder.add(port, "limelight.local", port);
        	}
		}

		// This is for advantagekit
		Logger.start();

		inst = NetworkTableInstance.getDefault();
		table = inst.getTable("Shuffleboard");

		gotoPositionSubscriber = table.getIntegerTopic("gotoPosition").subscribe(0); // default value
		gotoPositiionPublisher = table.getIntegerTopic("gotoPosition").publish(PubSubOption.keepDuplicates(false));
	}

	@Override
	public void robotPeriodic() {

		CommandScheduler.getInstance().run();

		/*Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
		Logger.recordOutput("Power/IsBrownedOut", RobotController.isBrownedOut());*/
		Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
		
		Logger.recordOutput("CAN/ReceiveErrorCount", RobotController.getCANStatus().receiveErrorCount);
		Logger.recordOutput("CAN/TransmitErrorCount", RobotController.getCANStatus().transmitErrorCount);
		Logger.recordOutput("CAN/PercentBusUtilization", RobotController.getCANStatus().percentBusUtilization);
		
		int p = (int) gotoPositionSubscriber.get();

		switch(p) {
			case -1:
				// cancel the goto pose
				robotContainer.gotoPoseCancel();
				gotoPositiionPublisher.set(0);
				break;
			case 0:
				// dont' do anything
				break;
			case 3:
				robotContainer.goToPose(Constants.PoseDefinitions.kFieldPoses.APRILTAG_3);
				gotoPositiionPublisher.set(0);
				break;
			case 2:
				break;
			case 4:
				robotContainer.goToPose(Constants.PoseDefinitions.kFieldPoses.APRILTAG_4);
				gotoPositiionPublisher.set(0);
				break;
			case 9:
				robotContainer.goToPose(Constants.PoseDefinitions.kFieldPoses.APRILTAG_9);
				gotoPositiionPublisher.set(0);
				break;
		}
	}

	@Override
	public void disabledInit() {
		armSubsystem = RobotContainer.armSubsystem;
		elevatorSystem = RobotContainer.elevatorSubsystem;
		armSubsystem.setDesiredState(ArmSubsystem.ArmState.Start);
		elevatorSystem.setDesiredState(ElevatorSubsystem.ElevatorState.Start);
	}

	@Override
	public void disabledPeriodic() {
		
	}

	@Override
	public void autonomousInit() {
		
		autonomousCommand = robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

	}

	@Override
	public void teleopPeriodic() {
		
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationInit() {
		
	}

	@Override
	public void simulationPeriodic() {
		
	}
}
