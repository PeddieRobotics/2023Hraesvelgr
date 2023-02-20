package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
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
        arm.setShoulderPosition(ShoulderConstants.kLLSeekAngle);
        arm.setWristPosition(WristConstants.kLLSeekAngle);
        // Needs logic for this...probably needs to make sure wrist is high enough before stowing the shoulder
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kLLSeekAngle) && arm.isWristAtAngle(WristConstants.kLLSeekAngle);
    }
}
