package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetLevelTwoConePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetLevelTwoConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setWristPosition(40);
        arm.setShoulderPositionSmartMotion(shoulder.getkL2ConeAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.MOVING);
        
    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-30)){
            arm.setWristPosition(wrist.getkL2ConeAngle());
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.setState(ArmState.L2_CONE);
        arm.holdShoulderPosition();

    }

    @Override
    public boolean isFinished() {
        return arm.isShoulderAtAngle(shoulder.getkL2ConeAngle()) && arm.isWristAtAngle(wrist.getkL2ConeAngle());
    }


}
