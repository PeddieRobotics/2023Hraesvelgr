package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetSingleSSConePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetSingleSSConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setShoulderPositionSmartMotion(shoulder.getkSingleSSConeAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setWristPosition(wrist.getkSingleSSConeAngle());
        arm.setState(ArmState.SINGLE_SS_CONE);
        arm.setGoalPose(ArmState.NONE);
        LimelightFront.getInstance().setPipeline(5); // Pipeline for cone detection from human player

    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkSingleSSConeAngle()) && arm.isWristAtAngle(wrist.getkSingleSSConeAngle());
    }


}