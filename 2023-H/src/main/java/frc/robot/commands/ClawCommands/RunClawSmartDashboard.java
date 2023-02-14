package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants.ClawConstants;

public class RunClawSmartDashboard extends CommandBase{
    private Claw claw;

    public RunClawSmartDashboard(){
        claw = Claw.getInstance();
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setSpeed(SmartDashboard.getNumber("OR: Claw Speed", 0));
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        // claw.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
