package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class AlignConeAfterIntake extends CommandBase{
    private Claw claw;
    private double initialTime, currentTime;

    public AlignConeAfterIntake(){
        claw = Claw.getInstance();

        addRequirements(claw);

    }

    @Override
    public void initialize() {
        claw.setSpeed(ClawConstants.kConeIntakeSpeed/10);

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
    }

    @Override
    public boolean isFinished() {
        return currentTime - initialTime > 1.0; // Give this cone alignment command a time limit
    }
        
}