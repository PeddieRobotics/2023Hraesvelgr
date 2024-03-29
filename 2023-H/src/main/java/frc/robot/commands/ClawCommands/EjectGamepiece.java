package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Logger;

public class EjectGamepiece extends Command{
    private Claw claw;
    private double initialTime, currentTime;
    //private Logger logger;

    public EjectGamepiece(){
        claw = Claw.getInstance();
        //logger = Logger.getInstance();

        addRequirements(claw);

    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();

        if(claw.getState() == ClawState.CONE){
            claw.outtakeCone();
        } else {
            claw.outtakeCube();
        }

        //logger.logCommand("Eject Gampiece", true);

    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        if(!claw.hasCone() && !claw.isJustEjectedGamepiece()){
            initialTime = Timer.getFPGATimestamp();
            claw.setEjectionTime(Timer.getFPGATimestamp());
            claw.setJustEjectedGamepiece(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
        claw.resetGamepieceAlignmentError();
        claw.setGamepieceOperatorOverride(false);
        claw.classifyGamepiece();
        claw.setCurrentAlignmentDistance(0.0);

        //logger.logCommand("Eject Gamepiece", false);
    }

    @Override
    public boolean isFinished() {
        if(Arm.getInstance().isInvertedL3() || Arm.getInstance().isL1Pose()){
            return currentTime - initialTime > 0.5;
        }
        else{
            return currentTime - initialTime > 1.0;
        }
    }
        
}