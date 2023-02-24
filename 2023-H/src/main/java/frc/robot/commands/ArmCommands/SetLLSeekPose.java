package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLLSeekPose extends CommandBase{
    private Arm arm;

    public SetLLSeekPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setWristPosition(40);
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(30)){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kLLSeekAngle);
        }

        if(arm.isShoulderBelowAngle(-55)){
            arm.setWristPosition(WristConstants.kLLSeekAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setState(ArmState.LL_SEEK);
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kLLSeekAngle) && arm.isWristAtAngle(WristConstants.kLLSeekAngle);
    }
}
