package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;

public class SetLevelThreeCubeInvertedPose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelThreeCubeInvertedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {

        if(arm.isShoulderBelowAngle(65)){
            arm.setShoulderPositionSmartMotion(shoulder.getkL3CubeInvertedAngle(), SmartMotionArmSpeed.FAST);
        }
        
        arm.setWristPosition(wrist.getkHomeAngle());
        arm.setState(ArmState.L3_CUBE_INVERTED);

    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(65.0)){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, 3000, 2000);
            arm.setShoulderPositionSmartMotion(shoulder.getkL3CubeInvertedAngle(), SmartMotionArmSpeed.SLOW);
        }

        if(arm.isShoulderAboveAngle(75.0)){
            arm.setWristPosition(wrist.getkL3CubeInvertedAngle());
        }
  
    }

    @Override
    public void end(boolean interrupted){
        arm.holdShoulderPosition();
        shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
        ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkL3CubeInvertedAngle()) && arm.isWristAtAngle(wrist.getkL3CubeInvertedAngle());
    }


}
