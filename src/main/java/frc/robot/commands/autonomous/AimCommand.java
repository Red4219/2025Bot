package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.tools.PhotonVision;
import frc.robot.subsystems.DriveSubsystem;

public class AimCommand extends Command {

    private static DriveSubsystem _driveSubsystem;
    private PhotonVision _photonVision;
    private int _targedNumber = 7;

    public AimCommand(PhotonVision photonVision, int target) {
         _driveSubsystem = RobotContainer.driveSubsystem;
         addRequirements(_driveSubsystem);
         _photonVision = photonVision;
         this._targedNumber = target;
    }

    @Override
    public void initialize() {
        if(DriverStation.getAlliance().isPresent()) {
            /*if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                _targedNumber = 7;
            } else {
                _targedNumber = 4;
            }*/
        }
    }

    @Override
    public void execute() {
        //System.out.println("calling aim command");
    }

    @Override
    public void end(boolean interrupted) {

    }

    public double calculateShooterHeight(double distance) {
        return distance;
    }

    public double calculateShooterSpeed(double distance) {
        return distance;
    }

    @Override
    public boolean isFinished() {

        if(!Constants.kEnablePhotonVision) {
            return true;
        }

        if(_photonVision == null) {
            System.out.println("AimCommand::isFinished() - _photonVision is null");
            return true;
        }

        if(_photonVision.canSeeTarget(_targedNumber) == false) {
            System.out.println("AimCommand::isFinished() - cannot see the target so returning true");
            // Stop because we cannot see the target
            return true;
        }

        double targetYaw = _photonVision.aimAtTarget(_targedNumber);

        if(targetYaw == 0.0) {
            return true;
        }

        if(Math.abs(targetYaw) > Constants.AutoConstants.kAimTargetTolerance) {

            if(targetYaw > 0) {
                _driveSubsystem.drive(0.0, 0.0, -.05);
            } else {
                _driveSubsystem.drive(0.0, 0.0, .05);
            }
            
            return false;
        }
        
        return true;
    }
    
}
