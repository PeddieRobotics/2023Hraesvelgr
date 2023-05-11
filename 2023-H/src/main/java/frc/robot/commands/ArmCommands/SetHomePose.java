package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;

public class SetHomePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private boolean wristHomed, shoulderHomed, shoulderHeld;
    private double initialShoulderMoveTime, currentShoulderMoveTime;

    private Blinkin blinkin;

    public SetHomePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        blinkin = Blinkin.getInstance();
        
    }

    @Override
    public void initialize() {
        arm.turnOffSmartLimits();
        arm.setState(ArmState.HOME);
        arm.setGoalPose(ArmState.NONE);

        blinkin.specialOperatorFunctionality();

        wristHomed = false;
        shoulderHomed = false;
        shoulderHeld = false;

        initialShoulderMoveTime = Timer.getFPGATimestamp();
        currentShoulderMoveTime = Timer.getFPGATimestamp();

        arm.setWristPercentOutput(0.3);
        arm.setShoulderPercentOutput(0.3);

    }

    @Override
    public void execute() {
        // currentShoulderMoveTime = Timer.getFPGATimestamp();

        // if(wrist.atLimitSensor()){
        //     arm.holdWristPosition();
        //     wristHomed = true;
        // }

        // if(currentShoulderMoveTime - initialShoulderMoveTime > 0.5 && !shoulderHeld){
        //     arm.holdShoulderPosition();
        //     shoulderHeld = true;
        // }

        // if(wristHomed && !shoulderHomed && shoulderHeld){
        //     arm.setShoulderPercentOutput(-0.3);
        // }

        // if(shoulder.atLimitSensor()){
        //     arm.holdShoulderPosition();
        //     shoulderHomed = true;
        // }
        
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            wrist.setEncoder(wrist.getkStowedAngle());
            shoulder.setEncoder(shoulder.getkStowedAngle());
            blinkin.success();
        } else{
            blinkin.failure();
        }

        arm.turnOnSmartLimits();

    }

    @Override
    public boolean isFinished() {
        return wristHomed && shoulderHomed;
    }


}
