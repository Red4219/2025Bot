package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlignLeftCommand;
import frc.robot.commands.AutoAlignRightCommand;
import frc.robot.mechanisms.LED.LEDStatus;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;
import frc.robot.tools.Limelight;

public class AutoAlignRightAutoCommand extends Command  {

    private boolean finished = false;
    private AutoAlignRightCommand autoAlignRightCommand;
    private SequentialCommandGroup scg;
    
    public AutoAlignRightAutoCommand() {
// <<<<<<< ElevatorWithNeos
        this.driveSubsystem = RobotContainer.driveSubsystem;
        this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
        this.limelight = RobotContainer.limelightL;
        this.armSubsystem = RobotContainer.armSubsystem;
        addRequirements(driveSubsystem);
        addRequirements(endEffectorSubsystem);
        addRequirements(armSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = false;
        autoAlignRightCommand = new AutoAlignRightCommand();
        scg = new SequentialCommandGroup(autoAlignRightCommand);
        scg.schedule();
        
        //CommandScheduler.getInstance().schedule(scg);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

// <<<<<<< ElevatorWithNeos
        if(limelight.hasTarget()) {
            aprilTagLocation = LimelightHelpers.getTX(Constants.LimelightConstants.name1);

            //System.out.println("aprilTagLocation: " + aprilTagLocation + " -Constants.DriveConstants.kAutoAlignOffset: " + (-Constants.DriveConstants.kAutoAlignOffset) + "Constants.DriveConstants.kAutoAlignTolerance")

            if(
                (aprilTagLocation ) > (Constants.DriveConstants.kAutoAlignRightOffset - Constants.DriveConstants.kAutoAlignTolerance)
                && (aprilTagLocation) < (Constants.DriveConstants.kAutoAlignRightOffset + Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                // We are in the zone
                driveSubsystem.driveRobotRelative(0.0, 0.0, 0.0);

                /*SequentialCommandGroup scg = new SequentialCommandGroup(
                    new EjectCoralCommand(),
                    new DriveBackwardsCommand(),
                    new ArmStartCommand()

                );

                scg.execute();*/

                // Set the LED to show that it has the target
                RobotContainer.led1.setStatus(LEDStatus.targetAquired);
                
                if(
                    armSubsystem.getDesiredState() == ArmState.CoralL1
                    || armSubsystem.getDesiredState() == ArmState.CoralL2
                ) {
                    // We are in a CoralL1 or CoralL2 position, eject out the front
                    endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);
                    finished = true;
                } else if(
                    armSubsystem.getDesiredState() == ArmState.CoralL3
                    || armSubsystem.getDesiredState() == ArmState.CoralL4
                ) {
                    // We are in a CoralL3 or CoralL4 position, eject out the back
                    endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralBack);
                    finished = true;
                }
            } else if(
                (aprilTagLocation) < (Constants.DriveConstants.kAutoAlignRightOffset + Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                driveSubsystem.driveRobotRelative(0.0, -Constants.DriveConstants.kAutoAlignSpeed, 0.0);

                // Set the LED to show that it has the target
                RobotContainer.led1.setStatus(LEDStatus.targetSearching);
            } else if (
                (aprilTagLocation) > (Constants.DriveConstants.kAutoAlignRightOffset - Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                driveSubsystem.driveRobotRelative(0.0, Constants.DriveConstants.kAutoAlignSpeed, 0.0);

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
        return scg.isFinished();
    }
}
