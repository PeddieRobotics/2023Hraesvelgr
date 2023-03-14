package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeCone extends CommandBase{
    private Blinkin blinkin;
    private Claw claw;
    private double initialTime, initialReverseTime, initialConeTime, currentTime;
    private boolean fixedCone, hasCone;

    public IntakeCone(){
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
        if(!hasCone && claw.hasCone()){
            initialConeTime = Timer.getFPGATimestamp();
            hasCone = true;
        }
        if(!fixedCone && hasCone && currentTime - initialConeTime > 0.1){
            initialReverseTime = Timer.getFPGATimestamp();
            claw.setSpeed(-ClawConstants.kConeSingleSSIntakeSpeed/10);
            fixedCone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted){
            if(claw.hasCube()){
                blinkin.success();
                claw.setSpeed(ClawConstants.kCubeHoldSpeed);
            }
            else if(claw.hasCone()){
                blinkin.success();
                claw.stopClaw();
                claw.monitorNewConeIntake();
            }
            else{
                blinkin.failure();
                claw.stopClaw();
            }
        }
        else{
            claw.setState(ClawState.EMPTY);     
            claw.stopClaw();
            blinkin.returnToRobotState();
        }
    }

    @Override
    public boolean isFinished() {
        return claw.hasGamepiece() && fixedCone && (currentTime - initialReverseTime) > 0.35;
    }

    
}
