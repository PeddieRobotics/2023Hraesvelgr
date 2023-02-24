package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetDoubleSSConePose extends CommandBase{
    private Arm arm;

    public SetDoubleSSConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPositionSmartMotion(ShoulderConstants.kDoubleSSConeAngle);
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-30)){
            arm.setWristPosition(WristConstants.kDoubleSSConeAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kDoubleSSConeAngle) && arm.isWristAtAngle(WristConstants.kDoubleSSConeAngle);
    }


}
