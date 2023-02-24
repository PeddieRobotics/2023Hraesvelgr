package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetStowedPose extends CommandBase{
    private Arm arm;
    private boolean transitory;

    public SetStowedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        transitory = false;
        
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kStowedAngle);
        arm.setState(ArmState.MOVING);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(WristConstants.kStowedAngle - 40) && !transitory){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kTransitoryAngle);
            transitory = true;
        }

        if(transitory && arm.isShoulderBelowAngle(-42)){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kStowedAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        transitory = false;
        arm.setState(ArmState.STOWED);
        arm.holdShoulderPosition();

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kStowedAngle) && arm.isShoulderAtAngle(ShoulderConstants.kStowedAngle);
    }


}
