package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetHumanPlayerConePose extends CommandBase{
    private Arm arm;

    public SetHumanPlayerConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderPosition(ShoulderConstants.kShoulderHumanPlayerConeAngle);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-30)){
            arm.setWristPosition(WristConstants.kWristHumanPlayerConeAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kShoulderHumanPlayerConeAngle) && arm.isWristAtAngle(WristConstants.kWristHumanPlayerConeAngle);
    }


}
