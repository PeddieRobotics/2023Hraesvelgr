package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants.ClawConstants;

public class AlignConeAfterIntake extends CommandBase{
    private Claw claw;
    private double initialTime, currentTime, mostRecentSwitchTime;

    public AlignConeAfterIntake(){
        claw = Claw.getInstance();

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.startMonitoringCurrent();

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();
        mostRecentSwitchTime = Timer.getFPGATimestamp();

        claw.setSpeed(-0.1);
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        if(currentTime - initialTime > 2.0){
            claw.setSpeed(-0.1);

        } else if(claw.getCurrentAverage() > 22 && currentTime - mostRecentSwitchTime > 0.2){
            claw.setSpeed(0.1);
            mostRecentSwitchTime = Timer.getFPGATimestamp();
        }
        else if(claw.getCurrentAverage() < 3 && currentTime - mostRecentSwitchTime > 0.2){
            claw.setSpeed(-0.1);
            mostRecentSwitchTime = Timer.getFPGATimestamp();
        }

        if(!claw.hasCone()){
            claw.setSpeed(ClawConstants.kConeIntakeSpeed/15);
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
        claw.stopMonitoringCurrent();
    }

    @Override
    public boolean isFinished() {
        return (currentTime-initialTime > 2.2) && claw.getCurrentAverage() > 22; // Give this cone alignment command a time limit
    }
        
}
