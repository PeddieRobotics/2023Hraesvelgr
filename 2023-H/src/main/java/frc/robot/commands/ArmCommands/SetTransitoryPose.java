package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetTransitoryPose extends Command{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetTransitoryPose() {
        arm = Arm.getInstance();
        addRequirements(arm);   

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(wrist.getkTransitoryAngle());
        arm.setState(ArmState.TRANSITORY);
        arm.setGoalPose(ArmState.NONE);

    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(shoulder.getkTransitoryAngle()) || arm.isShoulderBelowAngle(-65) || arm.isWristLessThanAngle(78)){
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
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
        return arm.isWristAtAngle(wrist.getkTransitoryAngle()) && arm.isShoulderAtAngle(shoulder.getkTransitoryAngle());
    }


}
