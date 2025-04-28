package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class EjectCoralNoCheck extends Command {

    private EndEffectorSubsystem endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
    private ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    private boolean finished = false;
    private double speed = 0.0;
    
    public EjectCoralNoCheck(double speed) {
        addRequirements(endEffectorSubsystem);
        addRequirements(armSubsystem);

        this.speed = speed;
    }

    @Override
    public void initialize() {
        finished = false;
        System.out.println("EjectCoralNoCheck::initialize() called");
    }

    @Override
    public void execute() {
        ArmState armState = armSubsystem.getDesiredState();

        if(armState == ArmState.CoralL1) {
            this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);
            //endEffectorSubsystem.setTargetVelocity1(1.0);
        } else if(armState == ArmState.CoralL2) {
            this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);
            //endEffectorSubsystem.setTargetVelocity1(1.0);
        } else if(armState == ArmState.CoralL3) {
            this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralBack);
            //endEffectorSubsystem.setTargetVelocity1(-1.0);
        } else if(armState == ArmState.CoralL4) {
            this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralBack);
            //endEffectorSubsystem.setTargetVelocity1(-1.0);
        } else if(armState == ArmState.Start) {
            this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);
        }

        //endEffectorSubsystem.setTargetVelocity1(1.0); 
        
        //finished = true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        //System.out.println("EjectCoralNoCheck::isFinished() called");
        return true;
    }
}
