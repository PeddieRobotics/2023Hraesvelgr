package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetTransitoryPose extends CommandBase{
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
        arm.setState(ArmState.MOVING);
    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(shoulder.getkTransitoryAngle()) || arm.isShoulderBelowAngle(-65) || arm.isWristAboveAngle(30)){
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
        }

    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.TRANSITION);
        arm.holdShoulderPosition();

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(wrist.getkTransitoryAngle()) && arm.isShoulderAtAngle(ShoulderConstants.kTransitoryAngle);
    }


}
