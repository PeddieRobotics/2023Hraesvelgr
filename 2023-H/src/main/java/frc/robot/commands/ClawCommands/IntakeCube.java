package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.ClawConstants;

public class IntakeCube extends CommandBase{
    private Claw claw;
    private double initialTime, currentTime;
    private boolean hasPiece;

    public IntakeCube(){
        claw = Claw.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        currentTime = 0.0;
    }

    @Override
    public void initialize() {
        claw.intakeCube();
        claw.setState(ClawState.INTAKING);     
        hasPiece = false;   
    }

    @Override
    public void execute() {
        if(claw.hasGamepiece() && !hasPiece){
            initialTime = Timer.getFPGATimestamp();
            hasPiece = true;
        } 
        currentTime = Timer.getFPGATimestamp();
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
        return hasPiece && (currentTime - initialTime) > 0.5;
    }

    
}
