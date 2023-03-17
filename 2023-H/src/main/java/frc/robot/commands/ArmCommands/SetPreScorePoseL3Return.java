package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;

public class SetPreScorePoseL3Return extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetPreScorePoseL3Return() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(100);
        arm.setState(ArmState.PRE_SCORE);
        arm.setGoalPose(ArmState.NONE);

        shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, 5500, 3000);
        arm.setShoulderPositionSmartMotion(shoulder.getkPreScoreAngle(), SmartMotionArmSpeed.SLOW);
    }

    @Override
    public void execute() {
        if(arm.isShoulderBelowAngle(80)){
            arm.setShoulderPositionSmartMotion(shoulder.getkPreScoreAngle(), SmartMotionArmSpeed.REGULAR);
            arm.setWristPosition(wrist.getkPreScoreAngle());
        }
        
    }

    @Override
    public void end(boolean interrupted){
        shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
        ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkPreScoreAngle()) && arm.isWristAtAngle(wrist.getkPreScoreAngle());
    }


}
