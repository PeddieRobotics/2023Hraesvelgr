package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelOnePose extends CommandBase{
    private Arm arm;

    public SetLevelOnePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setWristPosition(40);
    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(30)){
            arm.setShoulderPosition(ShoulderConstants.kL1Angle);
        }

        if(arm.isShoulderBelowAngle(-55)){
            arm.setWristPosition(WristConstants.kL1Angle);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kL1Angle) && arm.isShoulderAtAngle(ShoulderConstants.kL1Angle);
    }


}
