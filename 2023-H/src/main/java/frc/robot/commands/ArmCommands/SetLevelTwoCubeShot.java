package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelTwoCubeShot extends Command{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelTwoCubeShot() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kL3CubeForwardAngle);
        arm.setShoulderPositionSmartMotion(-3, SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.L2_CUBE);
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
        return arm.isShoulderAtAngle(-3)&&arm.isWristAtAngle(WristConstants.kL3CubeForwardAngle);
    }


}
