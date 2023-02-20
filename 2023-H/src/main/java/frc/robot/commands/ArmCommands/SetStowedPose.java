package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetStowedPose extends CommandBase{
    private Arm arm;

    public SetStowedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kStowedAngle);
    }

    @Override
    public void execute() {
        if(arm.isWristAtAngle(WristConstants.kStowedAngle)){
            arm.setShoulderPosition(ShoulderConstants.kStowedAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kStowedAngle) && arm.isShoulderAtAngle(ShoulderConstants.kStowedAngle);
    }


}
