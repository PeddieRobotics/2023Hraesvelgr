package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmCommands.SetTransitoryPoseL3Return;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Claw.ClawState;

public class EjectGamepiece extends CommandBase{
    private Claw claw;

    public EjectGamepiece(){
        claw = Claw.getInstance();
        addRequirements(claw);


    }

    @Override
    public void initialize() {
        if(claw.hasCone()){
            claw.outtakeCone();
        }
        else if(claw.hasCube()){
            claw.outtakeCube();
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return claw.getState() == ClawState.EMPTY;
    }

    
}
