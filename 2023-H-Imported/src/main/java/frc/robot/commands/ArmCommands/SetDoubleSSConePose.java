package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetDoubleSSConePose extends Command{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    private DriverOI oi;

    public SetDoubleSSConePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        arm.setShoulderPositionSmartMotion(shoulder.getkDoubleSSConeAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.DOUBLE_SS_CONE);
        arm.setGoalPose(ArmState.NONE);

        oi = DriverOI.getInstance();


    }

    @Override
    public void execute() {
        if(arm.isShoulderAboveAngle(-30)){
            arm.setWristPosition(wrist.getkDoubleSSConeAngle());

            if(oi.touchpadHeld()){
                arm.setWristPosition(wrist.getkDoubleSSConeAngle()+20);
            }
            else{
                arm.setWristPosition(wrist.getkDoubleSSConeAngle());
            }
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
        return false;
        // return arm.isShoulderAtAngle(shoulder.getkDoubleSSConeAngle()) && arm.isWristAtAngle(wrist.getkDoubleSSConeAngle());
    }


}
