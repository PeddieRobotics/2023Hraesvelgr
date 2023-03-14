package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;

public class SetWristHomePose extends CommandBase{
    private Arm arm;
    private Wrist wrist;
    private boolean wristMovedDown, wristHomed;
    private double initialWristMoveTime, currentWristMoveTime;

    private Blinkin blinkin;

    public SetWristHomePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        wrist = Wrist.getInstance();
        blinkin = Blinkin.getInstance();
        
    }

    @Override
    public void initialize() {
        wrist.turnOffSmartLimits();
        arm.setState(ArmState.HOME);
        arm.setGoalPose(ArmState.NONE);

        blinkin.homingArm();

        wristMovedDown = false;
        wristHomed = false;

        initialWristMoveTime = Timer.getFPGATimestamp();
        currentWristMoveTime = Timer.getFPGATimestamp();

        arm.setWristPercentOutput(-0.3);

    }

    @Override
    public void execute() {
        currentWristMoveTime = Timer.getFPGATimestamp();
        if(currentWristMoveTime - initialWristMoveTime > 0.5 && !wristMovedDown){
            arm.setWristPercentOutput(0.3);
            wristMovedDown = true;

        }

        if(wrist.atLimitSensor()){
            arm.setWristPercentOutput(0);
            wristHomed = true;
        }
        
    }

    @Override
    public void end(boolean interrupted){
        arm.setWristPercentOutput(0);
        
        if(!interrupted){
            wrist.setEncoder(wrist.getkStowedAngle());
            blinkin.success();
        }
        else{
            blinkin.failure();
        }

        wrist.turnOnSmartLimits();
    }

    @Override
    public boolean isFinished() {
        return wristHomed;
    }


}
