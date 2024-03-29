package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetTransitoryPoseL3Return extends Command{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetTransitoryPoseL3Return() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(wrist.getkHomeAngle());
        arm.setState(ArmState.TRANSITORY);
        arm.setGoalPose(ArmState.NONE);

        shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, 5500, 3000);
        arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.SLOW);
    }

    @Override
    public void execute() {
        if(arm.isShoulderBelowAngle(80)){
            arm.setWristPosition(wrist.getkTransitoryAngle());
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
        }
        
    }

    @Override
    public void end(boolean interrupted){
        shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
        ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderBelowAngle(0);
    }


}
