package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;

public class SetLevelThreeConePoseInAuto extends Command{
    private Arm arm;
    private Claw claw;
    private Shoulder shoulder;
    private Wrist wrist;
    private boolean finished;

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
        
        arm.setWristPosition(wrist.getkHomeAngle());
        arm.setState(ArmState.L3_CONE_INVERTED);
        arm.setGoalPose(ArmState.NONE);
        finished=false;

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
            finished=true;
        }
  
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished() {
        return finished;
    }


}
