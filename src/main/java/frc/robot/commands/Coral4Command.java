package frc.robot.commands;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class Coral4Command extends Command {

  ElevatorSubsystem elevatorSubsystem;
  ArmSubsystem armSubsystem;
  EndEffectorSubsystem endEffectorSubsystem;

  public Coral4Command() {
      this.elevatorSubsystem = RobotContainer.elevatorSubsystem;
      this.armSubsystem = RobotContainer.armSubsystem;
      this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
      
      addRequirements(elevatorSubsystem);
      addRequirements(armSubsystem);
      addRequirements(endEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    armSubsystem.setDesiredState(ArmState.CoralL4);
    
    elevatorSubsystem.setDesiredState(ElevatorState.CoralL4);
    endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);
    System.out.println("Coral4Command::execute() called");
    
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
