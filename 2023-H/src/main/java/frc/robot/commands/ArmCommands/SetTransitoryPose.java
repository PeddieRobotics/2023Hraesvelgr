package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetTransitoryPose extends CommandBase{
    private Arm arm;

    public SetTransitoryPose() {
        arm = Arm.getInstance();
        addRequirements(arm);   
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kStowedAngle);
        arm.setState(ArmState.MOVING);
    }

    @Override
    public void execute() {
        if(arm.isWristAtAngle(WristConstants.kStowedAngle)){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kTransitoryAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.TRANSITION);

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kStowedAngle) && arm.isShoulderAtAngle(ShoulderConstants.kTransitoryAngle);
    }


}
