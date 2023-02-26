package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
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
        if(arm.isShoulderAboveAngle(ShoulderConstants.kTransitoryAngle) || arm.isShoulderBelowAngle(-65) || arm.isWristAboveAngle(30)){
            arm.setShoulderPositionSmartMotion(ShoulderConstants.kTransitoryAngle, SmartMotionArmSpeed.REGULAR);
        }

    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.TRANSITION);
        arm.holdShoulderPosition();

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kStowedAngle) && arm.isShoulderAtAngle(ShoulderConstants.kTransitoryAngle);
    }


}
