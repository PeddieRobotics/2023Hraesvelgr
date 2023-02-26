package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelTwoConePose extends CommandBase{
    private Arm arm;

    public SetLevelTwoConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setWristPosition(40);
        arm.setShoulderPositionSmartMotion(ShoulderConstants.kL2ConeAngle);
        arm.setState(ArmState.MOVING);
        
    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-30)){
            arm.setWristPosition(WristConstants.kL2ConeAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.L2_CONE);
        arm.holdShoulderPosition();

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kL2ConeAngle) && arm.isWristAtAngle(WristConstants.kL2ConeAngle);
    }


}
