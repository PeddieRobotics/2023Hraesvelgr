package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetL3TransitoryPose extends CommandBase{
    private Arm arm;
    private boolean transitory;

    public SetL3TransitoryPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        transitory = false;
        
    }

    @Override
    public void initialize() {
        arm.setWristPosition(WristConstants.kStowedAngle);
        arm.setShoulderPosition(ShoulderConstants.kStowedAngle);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted){
        transitory = false;
    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(WristConstants.kStowedAngle) && arm.isShoulderBelowAngle(110);
    }


}
