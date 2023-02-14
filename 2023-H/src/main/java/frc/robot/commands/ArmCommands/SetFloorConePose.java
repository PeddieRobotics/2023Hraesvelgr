package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetFloorConePose extends CommandBase{
    private Arm arm;

    public SetFloorConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kShoulderFloorConeAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorConeAngle)){
            arm.setWristPosition(WristConstants.kWristFloorConeAngle);
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorConeAngle) && arm.isWristAtAngle(WristConstants.kWristFloorConeAngle);
    }


}
