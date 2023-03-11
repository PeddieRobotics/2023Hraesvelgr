package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.LimelightHelper;

public class SetPreScorePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private Claw claw;

    public SetPreScorePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        claw = Claw.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(wrist.getkPreScoreAngle());
        arm.setShoulderPositionSmartMotion(shoulder.getkPreScoreAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.PRE_SCORE);

        String limelightName;
        switch (arm.getGoalPose()) {
            case L3_CUBE_INVERTED:
                limelightName = "limelight-back";
                break;
            case L3_CONE_INVERTED:
                limelightName = "limelight-back";
                break;
            default:
                limelightName = "limelight-front";
        }

        if (claw.hasCone()) {
            LimelightHelper.setPipelineIndex(limelightName, 6); // Retroreflective tape pipeline
        } else {
            LimelightHelper.setPipelineIndex(limelightName, 0); // April tag pipeline
        }
        
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
        return arm.isShoulderAtAngle(shoulder.getkPreScoreAngle()) && arm.isWristAtAngle(wrist.getkPreScoreAngle());
    }


}
