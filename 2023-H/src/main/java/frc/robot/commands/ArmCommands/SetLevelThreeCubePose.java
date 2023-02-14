package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelThreeCubePose extends CommandBase{
    private Arm arm;

    public SetLevelThreeCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kShoulderLevelThreeCubeLobAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderLevelThreeCubeLobAngle)){
            arm.setWristPosition(WristConstants.kWristLevelThreeCubeLobAngle);
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderLevelThreeCubeLobAngle) && arm.isWristAtAngle(WristConstants.kWristLevelThreeCubeLobAngle);
    }


}
