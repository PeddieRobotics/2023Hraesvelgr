package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;

public class SetHomePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private boolean wristHomed, shoulderHomed, shoulderHeld;
    private double initialShoulderMoveTime, currentShoulderMoveTime;

    public SetHomePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        
    }

    @Override
    public void initialize() {
        arm.turnOffSmartLimits();
        arm.setState(ArmState.HOME);
        wristHomed = false;
        shoulderHomed = false;
        shoulderHeld = false;

        initialShoulderMoveTime = Timer.getFPGATimestamp();
        currentShoulderMoveTime = Timer.getFPGATimestamp();

        wrist.setPercentOutput(0.3);
        shoulder.setPercentOutput(0.3);

    }

    @Override
    public void execute() {
        currentShoulderMoveTime = Timer.getFPGATimestamp();

        if(wrist.atLimitSensor()){
            wrist.setPercentOutput(0);
            wristHomed = true;
        }

        if(currentShoulderMoveTime - initialShoulderMoveTime > 0.5 && !shoulderHeld){
            arm.holdShoulderPosition();
            shoulderHeld = true;
        }

        if(wristHomed && !shoulderHomed && currentShoulderMoveTime - initialShoulderMoveTime > 0.5){
            shoulder.setPercentOutput(-0.3);
        }

        if(shoulder.atLimitSensor()){
            shoulder.setPercentOutput(0);
            shoulderHomed = true;
        }
        
    }

    @Override
    public void end(boolean interrupted){
        wrist.setEncoder(103);
        shoulder.setEncoder(-75);
        arm.turnOnSmartLimits();
        arm.setState(ArmState.HOME);
        arm.setShoulderPercentOutput(0);
        wrist.setPercentOutput(0);

    }

    @Override
    public boolean isFinished() {
        return wristHomed && shoulderHomed;
    }


}
