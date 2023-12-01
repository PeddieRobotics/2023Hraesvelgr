package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.utils.Constants.ClawConstants;

public class NormalizeConeAfterIntake extends Command{
    private Claw claw;
    private double initialTime, currentTime;

    public NormalizeConeAfterIntake(){
        claw = Claw.getInstance();

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.startMonitoringCurrent();

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();

        claw.setSpeed(0.05);

        claw.setNormalizingCone(true);
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        if(currentTime - initialTime > 0.25){
            claw.setSpeed(-0.15);
        }

        if(!claw.hasCone()){
            claw.setSpeed(ClawConstants.kConeIntakeSpeed/5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
        claw.stopMonitoringCurrent();
        claw.setNormalizingCone(false);
    }

    @Override
    public boolean isFinished() {
        return claw.getCurrentAverage() > 22 || currentTime - initialTime > 3.0; // Give this cone alignment command a time limit
    }
        
}
