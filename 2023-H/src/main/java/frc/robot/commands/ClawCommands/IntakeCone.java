package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeCone extends CommandBase{
    private Claw claw;
    private double initialTime, currentTime;
    private boolean hasPiece;

    public IntakeCone(){
        claw = Claw.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        currentTime = 0.0;

    }

    @Override
    public void initialize() {
        claw.intakeCone();
        claw.setState(ClawState.INTAKING);

        initialTime = Timer.getFPGATimestamp();
        currentTime = initialTime;
        hasPiece = false;
    }

    @Override
    public void execute() {
        if(claw.hasGamepiece() && !hasPiece){
            initialTime = Timer.getFPGATimestamp();
            hasPiece = true;
        } 
        currentTime = Timer.getFPGATimestamp();

        if(claw.isFrontSensor() && (currentTime - initialTime) > 0.25){
            claw.setSpeed(ClawConstants.kConeIntakeSpeed/3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted){
            if(claw.hasCube()){
                claw.setSpeed(ClawConstants.kCubeHoldSpeed);
            }
            else{
                claw.stopClaw();
                claw.monitorNewConeIntake();
            }
        }
        else{
            claw.stopClaw();
        }
    }

    @Override
    public boolean isFinished() {
        return claw.hasGamepiece() && (currentTime - initialTime) > 0.75;
    }

    
}
