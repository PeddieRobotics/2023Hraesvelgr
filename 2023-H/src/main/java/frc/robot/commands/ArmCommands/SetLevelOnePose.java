package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelOnePose extends CommandBase{
    private Arm arm;

    public SetLevelOnePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kWristLevelOneAngle);
    }

    @Override
    public void execute() {
        if(arm.isWristAtAngle(WristConstants.kWristLevelOneAngle)){
            arm.setShoulderPosition(ShoulderConstants.kShoulderLevelOneAngle);
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kWristLevelOneAngle) && arm.isShoulderAtAngle(ShoulderConstants.kShoulderLevelOneAngle);
    }


}
