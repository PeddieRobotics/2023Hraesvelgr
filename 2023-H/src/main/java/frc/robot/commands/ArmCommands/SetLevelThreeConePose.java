package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelThreeConePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelThreeConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {

        if(arm.isShoulderAboveAngle(-45)){
            arm.setShoulderPositionSmartMotion(shoulder.getkL3ConeAngle(), SmartMotionArmSpeed.REGULAR);
        }
        
        arm.setWristPosition(103);
        arm.setState(ArmState.MOVING);

    }

    @Override
    public void execute() {
        if(arm.isWristAboveAngle(90)){
            arm.setShoulderPositionSmartMotion(shoulder.getkL3ConeAngle(), SmartMotionArmSpeed.REGULAR);
        }

        if(arm.isShoulderAboveAngle(100.0)){
            arm.setWristPosition(wrist.getkL3ConeAngle());
        }
  
    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.L3_CONE_INVERTED);
        arm.holdShoulderPosition();
    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkL3ConeAngle()) && arm.isWristAtAngle(wrist.getkL3ConeAngle());
    }


}
