package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetFloorCubePose extends CommandBase{
    private Arm arm;

    public SetFloorCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kShoulderFloorCubeAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorCubeAngle)){
            arm.setWristPosition(WristConstants.kWristFloorCubeAngle);
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderFloorCubeAngle) && arm.isWristAtAngle(WristConstants.kWristFloorCubeAngle);
    }


}

