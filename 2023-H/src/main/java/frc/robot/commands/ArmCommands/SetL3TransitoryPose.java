package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetL3TransitoryPose extends CommandBase{
    private Arm arm;
    private boolean transitory;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetL3TransitoryPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        transitory = false;

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(wrist.getkStowedAngle());
        arm.setShoulderPositionSmartMotion(shoulder.getkStowedAngle(), SmartMotionArmSpeed.FAST);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted){
        transitory = false;
    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(wrist.getkStowedAngle()) && arm.isShoulderBelowAngle(110);
    }


}
