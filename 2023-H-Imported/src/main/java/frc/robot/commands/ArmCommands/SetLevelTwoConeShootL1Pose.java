package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelTwoConeShootL1Pose extends Command{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private Claw claw;

    public SetLevelTwoConeShootL1Pose() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        addRequirements(arm, claw);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        claw.setSpeed(-1);
        arm.setWristPosition(130);
        arm.setShoulderPositionSmartMotion(20, SmartMotionArmSpeed.REGULAR);
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
        return arm.isShoulderAtAngle(20) && arm.isWristAtAngle(130);
    }


}
