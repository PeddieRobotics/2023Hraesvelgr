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
    private double initialTime, currentTime;
    private boolean hasPiece;

    public IntakeCone(){
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();
        addRequirements(claw);

        initialTime = 0.0;
        currentTime = 0.0;

    }

    @Override
    public void initialize() {
        claw.intakeCone();
        claw.setState(ClawState.INTAKING_CONE);

        initialTime = Timer.getFPGATimestamp();
        currentTime = initialTime;
        hasPiece = false;

        blinkin.intakingCone();
    }

    @Override
    public void execute() {
        if(!claw.hasGamepiece()){
            hasPiece = false;
        }

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
                blinkin.acquiredGamePiece();
                claw.setSpeed(ClawConstants.kCubeHoldSpeed);
            }
            else if(claw.hasCone()){
                blinkin.acquiredGamePiece();
                claw.stopClaw();
                claw.monitorNewConeIntake();
            }
            else{
                blinkin.failure();
                claw.stopClaw();
            }
        }
        else{
            claw.stopClaw();
            blinkin.black();
        }
    }

    @Override
    public boolean isFinished() {
        return hasPiece && (currentTime - initialTime) > 0.5;
    }

    
}
