package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;

public class SetShoulderHomePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private boolean shoulderMovedUp, shoulderHomed;
    private double initialShoulderMoveTime, currentShoulderMoveTime;

    private Blinkin blinkin;

    public SetShoulderHomePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        blinkin = Blinkin.getInstance();
        
    }

    @Override
    public void initialize() {
        shoulder.turnOffSmartLimits();
        arm.setState(ArmState.HOME);
        arm.setGoalPose(ArmState.NONE);

        blinkin.homingArm();

        shoulderMovedUp = false;
        shoulderHomed = false;

        initialShoulderMoveTime = Timer.getFPGATimestamp();
        currentShoulderMoveTime = Timer.getFPGATimestamp();

        shoulder.setPercentOutput(0.3);

    }

    @Override
    public void execute() {
        currentShoulderMoveTime = Timer.getFPGATimestamp();

        if(currentShoulderMoveTime - initialShoulderMoveTime > 0.5 && !shoulderMovedUp){
            shoulder.setPercentOutput(-0.3);
            shoulderMovedUp = true;

        }

        if(shoulder.atLimitSensor() && shoulderMovedUp){
            shoulder.setPercentOutput(0);
            shoulderHomed = true;
        }
        
    }

    @Override
    public void end(boolean interrupted){
        shoulder.setPercentOutput(0);

        shoulder.turnOnSmartLimits();
        
        shoulder.setEncoder(shoulder.getkHomeAngle());

        blinkin.returnToRobotState();
    }

    @Override
    public boolean isFinished() {
        return shoulderHomed;
    }


}
