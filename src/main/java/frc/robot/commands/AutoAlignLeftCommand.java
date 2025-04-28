package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.mechanisms.LED.LEDStatus;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;
import frc.robot.tools.Limelight;

public class AutoAlignLeftCommand extends Command {

    private DriveSubsystem driveSubsystem;
    private EndEffectorSubsystem endEffectorSubsystem;
    private ArmSubsystem armSubsystem;
    private Limelight limelight;
    private boolean finished = false;
    private double aprilTagLocation = 0.0;
    private double aprilTagLocationY = 0.0;

    public AutoAlignLeftCommand() {
        this.driveSubsystem = RobotContainer.driveSubsystem;
        this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
        this.limelight = RobotContainer.limelight;
        this.armSubsystem = RobotContainer.armSubsystem;
        addRequirements(driveSubsystem);
        addRequirements(endEffectorSubsystem);
        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(limelight.hasTarget()) {
            aprilTagLocation = LimelightHelpers.getTX(Constants.LimelightConstants.name);            
            int error = (int) (2 - aprilTagLocation);
            double kP = 0.005;

            if(Math.abs(error) <4) {
                // We are in the zone
                driveSubsystem.driveRobotRelative(0.0, 0.0, 0.0);

                // Set the LED to show that it has the target
                RobotContainer.led1.setStatus(LEDStatus.targetAquired);
                
                if(
                    armSubsystem.getDesiredState() == ArmState.CoralL1
                    || armSubsystem.getDesiredState() == ArmState.CoralL2
                ) {
                    // We are in a CoralL1 or CoralL2 position, eject out the front
                    endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);
                    //finished = true;
                } else if(
                    armSubsystem.getDesiredState() == ArmState.CoralL3
                    || armSubsystem.getDesiredState() == ArmState.CoralL4
                ) {
                    // We are in a CoralL3 or CoralL4 position, eject out the back
                    endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralBack);
                    //finished = true;
                }
            } else {
                driveSubsystem.driveRobotRelative(0.0, -error*kP, 0.0);

                // Set the LED to show that it has the target
                RobotContainer.led1.setStatus(LEDStatus.targetSearching);
            }
        } else {
            // we don't have a target to stop
            driveSubsystem.driveRobotRelative(0.0, 0.0, 0.0);

            // Set the LED to show that it has the target
            RobotContainer.led1.setStatus(LEDStatus.targetSearching);
        }
    
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}

