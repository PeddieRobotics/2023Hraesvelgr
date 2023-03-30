package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;

public class SetExtendedFloorCubePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private boolean approachFromAbove, overshotTargetAngle;

    public SetExtendedFloorCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();

        SmartDashboard.putNumber("intake accel", 6000);
    }

    @Override
    public void initialize() {
        overshotTargetAngle = false;

        if(arm.isShoulderAboveAngle(shoulder.getkExtendedFloorCubeAngle())){
            approachFromAbove = true;
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
            arm.setWristPosition(88);
        }
        else{
            approachFromAbove = false;
            arm.setWristPosition(wrist.getkExtendedFloorCubeAngle());
            shoulder.setRegularSmartMotionParameters(ShoulderConstants.kSmartMotionRegularSetpointTol,
            ShoulderConstants.kSmartMotionRegularMinVel, ShoulderConstants.kSmartMotionRegularMaxVel, SmartDashboard.getNumber("intake accel", 6000));
            // arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
        }

        arm.setState(ArmState.FLOOR_INTAKE_CONE_EXTENDED);
        arm.setGoalPose(ArmState.NONE);
    }

    @Override
    public void execute() {
        if(arm.isWristGreaterThanAngle(wrist.getkL1Angle())){
            arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.REGULAR);
        }

        if(overshotTargetAngle){
            arm.setWristPosition(wrist.getkExtendedFloorCubeAngle());
            arm.setShoulderPositionSmartMotion(shoulder.getkExtendedFloorCubeAngle(), SmartMotionArmSpeed.SLOW);
        }

        if(approachFromAbove && arm.isShoulderAtAngle(shoulder.getkTransitoryAngle())){
            overshotTargetAngle = true;
        }

    }

    @Override
    public void end(boolean interrupted){
        shoulder.setRegularSmartMotionParameters(ShoulderConstants.kSmartMotionRegularSetpointTol,
        ShoulderConstants.kSmartMotionRegularMinVel, ShoulderConstants.kSmartMotionRegularMaxVel, ShoulderConstants.kSmartMotionRegularMaxAccel);

        if(!interrupted){
            arm.holdShoulderPosition();
        }
    }

    @Override
    public boolean isFinished() {
        if(!approachFromAbove){
            return arm.isShoulderAtAngle(shoulder.getkExtendedFloorCubeAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle());
        }
        else if(approachFromAbove && overshotTargetAngle){
            return arm.isShoulderAtAngle(shoulder.getkExtendedFloorCubeAngle()) && arm.isWristAtAngle(wrist.getkExtendedFloorCubeAngle());
        }
        else{
            return false;
        }
    }


}