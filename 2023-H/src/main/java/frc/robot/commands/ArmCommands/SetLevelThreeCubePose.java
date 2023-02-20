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
        arm.setShoulderPosition(ShoulderConstants.kL3CubeInvertedAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-30)){
            arm.setWristPosition(WristConstants.kL3CubeForwardAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kL3CubeInvertedAngle) && arm.isWristAtAngle(WristConstants.kL3CubeForwardAngle);
    }


}
