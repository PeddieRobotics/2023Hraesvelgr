package frc.robot.commands.ArmCommands;

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelThreeConePose extends CommandBase{
    private Arm arm;

    public SetLevelThreeConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kShoulderLevelThreeConeAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderLevelThreeConeAngle)){
            arm.setWristPosition(WristConstants.kWristLevelThreeConeAngle);
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderLevelThreeConeAngle) && arm.isWristAtAngle(WristConstants.kWristLevelThreeConeAngle);
    }


}
