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
        arm.setShoulderPositionSmartMotion(shoulder.getkSingleSSAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setWristPosition(wrist.getkSingleSSAngle());
        arm.setState(ArmState.STOWED);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            arm.setState(ArmState.STOWED);
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkSingleSSAngle()) && arm.isWristAtAngle(wrist.getkSingleSSAngle());
    }


}
