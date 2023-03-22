package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;

public class SetLevelThreeConePoseInAuto extends CommandBase{
    private Arm arm;
    private Claw claw;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelThreeConePoseInAuto() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();

        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        if(arm.isShoulderBelowAngle(65)){
            arm.setShoulderPositionSmartMotion(shoulder.getkL3ConeInvertedAngle(), SmartMotionArmSpeed.FAST);
        }
        
        arm.setWristPosition(13);
        arm.setState(ArmState.L3_CONE_INVERTED);
        arm.setGoalPose(ArmState.NONE);

    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(65.0)){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, 3000, 2000);
            arm.setShoulderPositionSmartMotion(shoulder.getkL3ConeInvertedAngle(), SmartMotionArmSpeed.SLOW);
        }

        if(arm.isShoulderAboveAngle(75.0)){
            arm.setWristPosition(wrist.getkL3ConeInvertedAngle());
        }

        if(arm.isShoulderAboveAngle(153)){
            claw.outtakeCone();
        }
  
    }

    @Override
    public void end(boolean interrupted){
        claw.stopClaw();
        
        shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
        ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);

        arm.setWristPosition(wrist.getkHomeAngle()); // prepare the wrist to return
    }

    @Override
    public boolean isFinished() {
        return !claw.hasGamepiece(); // Once we've released the game piece, we can stop immediately so that we can rotate the arm back
    }


}
