package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelTwoConeStowedPose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelTwoConeStowedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kHomeAngle);
        arm.setShoulderPositionSmartMotion(12.5, SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.L2_CONE);
        arm.setGoalPose(ArmState.NONE);

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(12.5);
    }


}
