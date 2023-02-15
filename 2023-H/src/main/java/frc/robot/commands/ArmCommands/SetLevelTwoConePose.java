package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelTwoConePose extends CommandBase{
    private Arm arm;

    public SetLevelTwoConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kShoulderLevelTwoConeAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderLevelTwoConeAngle)){
            arm.setWristPosition(WristConstants.kWristLevelTwoConeAngle);
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderLevelTwoConeAngle) && arm.isWristAtAngle(WristConstants.kWristLevelTwoConeAngle);
    }


}
