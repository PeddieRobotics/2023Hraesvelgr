package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ConeIntake extends CommandBase{
    private Claw claw;

    public ConeIntake(){
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
        return claw.hasCone();
    }

    
}
