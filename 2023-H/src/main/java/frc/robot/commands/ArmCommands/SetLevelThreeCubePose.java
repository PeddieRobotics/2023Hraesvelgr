package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
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
        arm.setWristPosition(30);
        arm.setShoulderPosition(ShoulderConstants.kL3CubeForwardAngle);
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-20)){
            arm.setWristPosition(WristConstants.kL3CubeForwardAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.L3_CUBE_FORWARD);
        arm.holdShoulderPosition();

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kL3CubeForwardAngle) && arm.isWristAtAngle(WristConstants.kL3CubeForwardAngle);
    }


}
