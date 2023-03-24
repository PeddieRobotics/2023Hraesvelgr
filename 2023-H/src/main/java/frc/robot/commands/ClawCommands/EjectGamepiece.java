package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;

public class EjectGamepiece extends CommandBase{
    private Claw claw;
    private double initialTime, currentTime;

    private Blinkin blinkin;

    public EjectGamepiece(){
        claw = Claw.getInstance();
        blinkin = Blinkin.getInstance();

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
        if(Arm.getInstance().isArmScoringPose()){
            blinkin.success();
        }

    }

    @Override
    public boolean isFinished() {
        return currentTime - initialTime > 0.5;
    }
        
}