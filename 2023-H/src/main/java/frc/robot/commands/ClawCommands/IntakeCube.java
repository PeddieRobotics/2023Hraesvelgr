package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeCube extends CommandBase{
    private Claw claw;

    public IntakeCube(){
        claw = Claw.getInstance();

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.intakeCube();
        
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted){
            claw.setSpeed(ClawConstants.kCubeHoldSpeed);
        }
        else{
            claw.stopClaw();
        }
    }

    @Override
    public boolean isFinished() {
        return claw.hasCube();

    }

    
}
