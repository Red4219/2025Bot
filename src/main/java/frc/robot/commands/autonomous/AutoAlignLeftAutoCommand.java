package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlignLeftCommand;
import frc.robot.mechanisms.LED.LEDStatus;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;
import frc.robot.tools.Limelight;

public class AutoAlignLeftAutoCommand extends Command {

    private boolean finished = false;
    private AutoAlignLeftCommand autoAlignLeftCommand;
    private SequentialCommandGroup scg;


    public AutoAlignLeftAutoCommand() {
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = false;
        autoAlignLeftCommand = new AutoAlignLeftCommand();
        scg = new SequentialCommandGroup(autoAlignLeftCommand);
        scg.schedule();
        
        //CommandScheduler.getInstance().schedule(scg);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        scg.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return scg.isFinished();
    }
}
