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

    private boolean wristHomed, shoulderHomed;
    private double initialTime, currentTime;

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

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();

        wrist.startMonitoringCurrent();
        shoulder.startMonitoringCurrent();

        arm.setWristPercentOutput(-0.3);
        arm.setShoulderPercentOutput(-0.3);

    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        if(wrist.getCurrentAverage() > 18.0 && !wristHomed){
            arm.holdWristPosition();
            wristHomed = true;
        }

        if(shoulder.getCurrentAverage() > 58.0 && !shoulderHomed){
            shoulder.setEncoder(shoulder.getkHomeAngle());
            arm.holdShoulderPosition();
            shoulderHomed = true;
        }
        
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            blinkin.success();
        } else{
            blinkin.failure();
        }

        arm.turnOnSmartLimits();
        wrist.stopMonitoringCurrent();
        shoulder.stopMonitoringCurrent();

    }

    @Override
    public boolean isFinished() {
        return wristHomed && shoulderHomed;
    }


}
