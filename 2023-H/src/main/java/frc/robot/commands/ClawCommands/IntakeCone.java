package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;

public class IntakeCone extends CommandBase{
    private Claw claw;

    public IntakeCone(){
        claw = Claw.getInstance();
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.intakeCone();
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
        // if(claw.monitor()){
        //     claw.setState(ClawState.CONE);
        //     return true;
        // }
        return false;
    }

    
}
