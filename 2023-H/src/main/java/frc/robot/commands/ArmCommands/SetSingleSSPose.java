package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetSingleSSPose extends CommandBase{
    private Arm arm;

    public SetSingleSSPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kSingleSSAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-30)){
            arm.setWristPosition(WristConstants.kSingleSSAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kSingleSSAngle) && arm.isWristAtAngle(WristConstants.kSingleSSAngle);
    }


}