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

        claw.setEjectionTime(Timer.getFPGATimestamp());
        claw.setJustEjectedGamepiece(true);

        if(claw.getState() == ClawState.CONE){
            claw.outtakeCube();
        } else {
            claw.stopClaw(); // Stop the motor just before ejection on cubes to minimize weird "pop-out" behaviors
        }

    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        // We are now good to eject the cube (200 ms delay)
        if(currentTime - initialTime > 0.2 && claw.getState() == ClawState.CUBE){
            claw.outtakeCube();
        }
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopClaw();
        claw.resetGamepieceAlignmentError();
        if(Arm.getInstance().isArmScoringPose()){
            blinkin.success();
        }

    }

    @Override
    public boolean isFinished() {
        return currentTime - initialTime > 0.5;
    }
        
}