package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.LimelightHelper;

public class SetPreScorePoseWristDown extends Command{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private Claw claw;

    private ArmState previousState;

    public SetPreScorePoseWristDown() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        claw = Claw.getInstance();
    }

    @Override
    public void initialize() {
        previousState = arm.getState();

        arm.setWristPosition(wrist.getkExtendedFloorCubeAngle());
        if(previousState != ArmState.L2_CONE){
            arm.setShoulderPositionSmartMotion(shoulder.getkPreScoreAngle(), SmartMotionArmSpeed.REGULAR);
        }
        arm.setState(ArmState.PRE_SCORE);

        claw.prepareLimelightForScoring();
        
    }

    @Override
    public void execute() {
        if(previousState == ArmState.L2_CONE){
            if(arm.isWristLessThanAngle(78)){
                arm.setShoulderPositionSmartMotion(shoulder.getkPreScoreAngle(), SmartMotionArmSpeed.SLOW);
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            arm.holdShoulderPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkPreScoreAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle());
    }


}
