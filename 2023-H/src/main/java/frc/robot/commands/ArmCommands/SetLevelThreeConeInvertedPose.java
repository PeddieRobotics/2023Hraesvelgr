package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.DriverOI.DriveSpeedMode;

public class SetLevelThreeConeInvertedPose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelThreeConeInvertedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        DriverOI.getInstance().setDriveSpeedMode(DriveSpeedMode.SLOW);

        if(arm.isShoulderBelowAngle(65)){
            arm.setShoulderPositionSmartMotion(shoulder.getkL3ConeInvertedAngle(), SmartMotionArmSpeed.FAST);
        }
        
        arm.setWristPosition(wrist.getkHomeAngle());
        arm.setState(ArmState.L3_CONE_INVERTED);
        arm.setGoalPose(ArmState.NONE);


    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(65)){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, 3000, 2000);
            arm.setShoulderPositionSmartMotion(shoulder.getkL3ConeInvertedAngle(), SmartMotionArmSpeed.SLOW);
        }

        if(arm.isShoulderAboveAngle(75.0)){
            arm.setWristPosition(wrist.getkL3ConeInvertedAngle());
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
        return arm.isShoulderAtAngle(shoulder.getkL3ConeInvertedAngle()) && arm.isWristAtAngle(wrist.getkL3ConeInvertedAngle());
    }


}
