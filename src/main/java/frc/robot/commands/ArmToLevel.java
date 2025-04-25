package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;



public class ArmToLevel extends Command {
    ArmSubsystem armSubsystem;
    ArmSubsystem.ArmState state;
    // Constructor

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.armSubsystem = RobotContainer.armSubsystem;
    this.state = ArmState.CoralL4; // Set the desired arm state here
    addRequirements(armSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setDesiredState(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atTargetPosition();
  }
}