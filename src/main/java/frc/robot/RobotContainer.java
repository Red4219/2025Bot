package frc.robot;

import java.util.OptionalLong;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.tools.JoystickUtils;
import frc.robot.tools.Limelight;
import frc.robot.tools.PhotonVision;
import frc.robot.tools.parts.PathBuilder;
import frc.robot.Constants.OperatorConstants;
import frc.robot.mechanisms.LED;
import frc.robot.mechanisms.LED.LEDStatus;
import frc.robot.commands.AlgaeFloorCommand;
import frc.robot.commands.ArmStartCommand;
import frc.robot.commands.AutoAlignLeftCommand;
import frc.robot.commands.AutoAlignRightCommand;
import frc.robot.commands.Coral1Command;
import frc.robot.commands.Coral2Command;
import frc.robot.commands.Coral3Command;
import frc.robot.commands.Coral4Command;
import frc.robot.commands.CoralHumanCommand;
import frc.robot.commands.EjectAlgaeCommand;
import frc.robot.commands.EjectCoralReverse;
import frc.robot.commands.IntakeNoWait;
import frc.robot.commands.StartCommand;
import frc.robot.commands.StopIntake;
import frc.robot.commands.autonomous.DelayCommand;
import frc.robot.commands.autonomous.EjectCoralCommand;
import frc.robot.commands.autonomous.IntakeCoralWaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

	public static final Field2d field = new Field2d();
	public static final PhotonVision photonVision = new PhotonVision();
	public static final Limelight limelight = new Limelight();
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public static final ArmSubsystem armSubsystem = new ArmSubsystem();
	public static final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
	public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
	public static final LED led1 = new LED(0);
	//private static final CommandXboxController operatorController = new CommandXboxController(1);

	
	// This is required by pathplanner
	public final static PathBuilder autoBuilder = new PathBuilder();

	private final CommandXboxController driverController = new CommandXboxController(0);
	//private final CommandXboxController programmerController = new CommandXboxController(
			//OperatorConstants.kProgrammerControllerPort);
	private final CommandXboxController operatorController = new CommandXboxController(1);

	private SendableChooser<Command> autoChooser = new SendableChooser<>();

	//private boolean setupAuto = false;

	public SendableChooser<Command> getAutoChooser() {
		return autoChooser;
	}

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer(boolean isSim) {

		// Configure the trigger bindings
		configureBindings();

		
		// Register commands to be used in Auto
		NamedCommands.registerCommand("IntakeCoralWait", new IntakeCoralWaitCommand());
		NamedCommands.registerCommand("IntakeNoWait", new IntakeNoWait());
		NamedCommands.registerCommand("Delay500", new DelayCommand(OptionalLong.of(500)));
		NamedCommands.registerCommand("EjectCoral500", new EjectCoralCommand());
		NamedCommands.registerCommand("EjectCoral", new EjectCoralCommand());
		NamedCommands.registerCommand("EjectCoralReverse500", new EjectCoralReverse(OptionalLong.of(500)));
		NamedCommands.registerCommand("Coral4Command", new Coral4Command());
		NamedCommands.registerCommand("Coral3Command", new Coral3Command());
		NamedCommands.registerCommand("Coral2Command", new Coral2Command());
		NamedCommands.registerCommand("Coral1Command", new Coral1Command());

		/*if(Constants.kEnablePhotonVision) {
			NamedCommands.registerCommand("Aim", new AimCommand(photonVision));
		} else {
			NamedCommands.registerCommand("Aim", new DummyCommand());
		}*/
		
		// This creates the chooser from the autos built in Autonomous
		//autoChooser = AutoBuilder.buildAutoChooser();

		// need to fix this
		autoChooser = AutoBuilder.buildAutoChooser("Auto 1");

		SmartDashboard.putData("Auto", autoChooser);
		//SmartDashboard.putData(autoChooser);

		// Add the chooser to the Shuffleboard to select which Auo to run
		Shuffleboard.getTab("Autonomous").add("Auto", autoChooser)
		.withWidget(BuiltInWidgets.kComboBoxChooser);

		//autoChooser.onChange(RobotContainer::selected);

		led1.setStatus(LEDStatus.ready);
	}

	/*public void setupAuto(boolean setupAuto) {
		//this.setupAuto = setupAuto;
		RobotContainer.driveSubsystem.setupAuto(setupAuto);
	}*/

	// this is for testing
	/*private static void selected(Command c) {
		
		try {
			if(c instanceof PathPlannerAuto) {
				PathPlannerAuto p = (PathPlannerAuto) c;
				var t = PathPlannerAuto.getPathGroupFromAutoFile(p.getName()).get(0).getAllPathPoints().get(0);

				if(t != null && t.position != null && t.rotationTarget != null) {
					System.out.println("Name: " + p.getName() + " x:" + t.position.getX() + " y: " + t.position.getY() + " rotation: " + t.rotationTarget.rotation().getDegrees());
					RobotContainer.driveSubsystem.setStartPosition(new Pose2d(t.position, t.rotationTarget.rotation()));
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
	}*/

	private void configureBindings() {

		// Always point the robot at the target
		//operatorController.button(2).onTrue(
		/*operatorController.leftTrigger().onTrue(
			Commands.parallel(new ChassisAimCommand(), new ArmAimCommand())			
		);*/

		//operatorController.button(3).whileTrue(new RunCommand(() -> driveSubsystem.goToPose(Constants.PoseDefinitions.kFieldPoses.AMPLIFIER)));
		//operatorController.button(4).whileTrue(new RunCommand(() -> driveSubsystem.goToPose(Constants.PoseDefinitions.kFieldPoses.SOURCE)));

		if(RobotBase.isReal()) {
			// Real, not a simulation
			operatorController.button(3).whileTrue(new Coral1Command());
			operatorController.button(4).whileTrue(new Coral2Command());
			operatorController.button(2).whileTrue(new Coral3Command());
			operatorController.button(10).whileTrue(new Coral4Command());
			
			//driverController.button(3).whileTrue(new StartCommand());

			// Intake coral from human element
			operatorController.button(8).whileTrue(new CoralHumanCommand());

			// This is for the floor algae
			operatorController.button(9).whileTrue(new AlgaeFloorCommand());
			//operatorController.button(9).whileFalse(new StopIntake());

			operatorController.leftTrigger().whileTrue(new EjectAlgaeCommand());
			//operatorController.button(9).whileFalse(new StopIntake());


			driverController.leftBumper().whileTrue(new AutoAlignLeftCommand());
			driverController.rightBumper().whileTrue(new AutoAlignRightCommand());

			operatorController.button(1).whileTrue(new SequentialCommandGroup(
				//new RunCommand(() -> armSubsystem.setDesiredState(ArmState.Start)),
				new ArmStartCommand(),
				//new RunCommand(() -> new DelayCommand(OptionalLong.of(500))),
				//new WaitCommand(1.0),
				new RunCommand(() -> elevatorSubsystem.setDesiredState(ElevatorState.Start))
			));
			//driverController.rightBumper().whileTrue(new RunCommand(() -> new SequentialCommandGroup(new IntakeNoWait(), new StopIntake()).execute()));

			//driverController.rightTrigger().whileTrue(new IntakeNoWait());
			//driverController.rightTrigger().whileFalse(new StopIntake());

			//driverController.rightTrigger().whileTrue(new EjectCoralCommand(null));

			operatorController.rightTrigger().whileTrue(new SequentialCommandGroup(
				new EjectCoralCommand()//,
				//new ArmStartCommand(), // this will retract the arm and stop end effector
				//new StartCommand() // this will retract the arm and move the elevator down
			));

			

			//driverController.rightTrigger().whileFalse(new StopEjectCoralCommand());
			

			
			//driverController.button(1).whileTrue(new RunCommand(() -> new ResetPositionCommand()));

			//driverController.button(1).whileTrue(new RunCommand(() -> sliderSubsystem.setDesiredState(ElevatorSubsystem.ElevatorState.Start)));

			// Swerve Drive method is set as default for drive subsystem
			driveSubsystem.setDefaultCommand(

				new RunCommand(() -> driveSubsystem.drive(
					JoystickUtils.processJoystickInput(driverController.getLeftY()),
					JoystickUtils.processJoystickInput(driverController.getLeftX()),
					JoystickUtils.processJoystickInput(-driverController.getRightX())
				),
				driveSubsystem
			)
			);
			
		} else {
			
			driverController.button(1).whileTrue(new AutoAlignRightCommand());
			
			// Swerve Drive method is set as default for drive subsystem
			driveSubsystem.setDefaultCommand(

				//new RunCommand(() -> driveSubsystem.drive(
				new RunCommand(() -> driveSubsystem.driveRobotRelative(
						JoystickUtils.processJoystickInput(driverController.getLeftY()),
						JoystickUtils.processJoystickInput(driverController.getLeftX()),
						//JoystickUtils.processJoystickInput(-operatorController.getRightX())
						//JoystickUtils.processJoystickInput(-operatorController.getRightX())
						JoystickUtils.processJoystickInput(-driverController.getRawAxis(2))
						//JoystickUtils.processJoystickInput(driverController.getRightY())
					),
					driveSubsystem
				)
			);
		}
		

		// Swerve Drive method is set as default for drive subsystem
		/*driveSubsystem.setDefaultCommand(

				new RunCommand(() -> driveSubsystem.drive(
					JoystickUtils.processJoystickInput(driverController.getLeftY()),
					JoystickUtils.processJoystickInput(driverController.getLeftX()),
					JoystickUtils.processJoystickInput(-driverController.getRightX())
				),
				driveSubsystem
			)
		);*/
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}