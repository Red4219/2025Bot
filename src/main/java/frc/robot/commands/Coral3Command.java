package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class Coral3Command extends Command {
  ElevatorSubsystem sliderSubsystem;
  ArmSubsystem armSubsystem;
  EndEffectorSubsystem endEffectorSubsystem;

  public Coral3Command() {
      this.sliderSubsystem = RobotContainer.elevatorSubsystem;
      this.armSubsystem = RobotContainer.armSubsystem;
      this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
      
      addRequirements(sliderSubsystem);
      addRequirements(armSubsystem);
      addRequirements(endEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setDesiredState(ArmState.CoralL3);
    new RunCommand(() -> new WaitCommand(2));
    sliderSubsystem.setDesiredState(ElevatorState.CoralL3);
    endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);

    System.out.println("Coral1Command::execute() called");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
