package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetHumanPlayerCubePose extends CommandBase{
    private Arm arm;

    public SetHumanPlayerCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kShoulderHumanPlayerCubeAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAtAngle(ShoulderConstants.kShoulderHumanPlayerCubeAngle)){
            arm.setWristPosition(WristConstants.kWristHumanPlayerCubeAngle);
        }
    }

    @Override
    public void end(boolean interupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderHumanPlayerCubeAngle) && arm.isWristAtAngle(WristConstants.kWristHumanPlayerCubeAngle);
    }


}
