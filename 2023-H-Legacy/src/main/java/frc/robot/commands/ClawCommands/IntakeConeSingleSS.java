package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeConeSingleSS extends CommandBase{
    private Blinkin blinkin;
    private Claw claw;
    private double initialTime, initialReverseTime, initialConeTime, currentTime;
    private boolean fixedCone, hasCone;

    public IntakeConeSingleSS(){
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        initialReverseTime = 0.0;
        initialConeTime = 0.0;
        currentTime = 0.0;

    }

    @Override
    public void initialize() {
        claw.intakeCone();
        claw.setState(ClawState.INTAKING_CONE);

        hasCone = false;
        fixedCone = false;

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();

        blinkin.intakingCone();
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        if(!hasCone && claw.isBackSensor()){
            initialConeTime = Timer.getFPGATimestamp();
            hasCone = true;
            Blinkin.getInstance().success();
        }
        if(!fixedCone && hasCone && currentTime - initialConeTime > 0.1){
            initialReverseTime = Timer.getFPGATimestamp();
            claw.setSpeed(-ClawConstants.kConeSingleSSIntakeSpeed/10);
            fixedCone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.classifyGamepiece();
    }

    @Override
    public boolean isFinished() {
        return claw.isEitherSensor() && fixedCone && (currentTime - initialReverseTime) > 0.35;
    }

    
}
