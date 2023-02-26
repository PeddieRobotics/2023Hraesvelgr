package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetSingleSSPose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetSingleSSPose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setShoulderPositionSmartMotion(ShoulderConstants.kSingleSSAngle, SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.MOVING);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-30)){
            arm.setWristPosition(WristConstants.kSingleSSAngle);
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.STOWED);
        arm.holdShoulderPosition();

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(ShoulderConstants.kSingleSSAngle) && arm.isWristAtAngle(WristConstants.kSingleSSAngle);
    }


}
