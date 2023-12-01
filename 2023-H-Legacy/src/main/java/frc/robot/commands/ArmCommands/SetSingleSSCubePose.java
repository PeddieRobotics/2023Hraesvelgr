package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;

public class SetSingleSSCubePose extends CommandBase{
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;

    public SetSingleSSCubePose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
    }

    @Override
    public void initialize() {
        // Determine the speed of the final stow based on where we are coming from
        if(arm.getState() == ArmState.FLOOR_INTAKE_CONE_EXTENDED){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kFloorIntakeConeMaxVelToStow, ShoulderConstants.kFloorIntakeConeMaxAccelToStow);
        }
        else if(arm.getState() == ArmState.FLOOR_INTAKE_CUBE_EXTENDED){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kFloorIntakeCubeMaxVelToStow, ShoulderConstants.kFloorIntakeCubeMaxAccelToStow);          
        }
        else if(arm.isArmScoringPose() || arm.isPreScorePose()){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kScoringPoseMaxVelToStow, ShoulderConstants.kScoringPoseMaxAccelToStow);
        }

        arm.setState(ArmState.SINGLE_SS_CUBE);
        arm.setGoalPose(ArmState.NONE);
        LimelightFront.getInstance().setPipeline(5); // Pipeline for cone detection from human player
        
        // Normal speed should be fine to move the arm towards the stowed position.
        // Motion profile to transitory and we'll switch when the shoulder gets lower.
        if(arm.isShoulderAboveAngle(-20)){
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
        }
        arm.setWristPosition(wrist.getkSingleSSCubeAngle());
    }

    @Override
    public void execute() {
        // Switch profiles into something slower so we don't smash the arm into the fulcrum.
        if (arm.isShoulderBelowAngle(-20)) {
            arm.setShoulderPositionSmartMotion(shoulder.getkStowedAngle(), SmartMotionArmSpeed.SLOW);
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
        return arm.isShoulderAtAngle(shoulder.getkSingleSSCubeAngle()) && arm.isWristAtAngle(wrist.getkSingleSSCubeAngle());
    }


}
