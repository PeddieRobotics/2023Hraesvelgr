package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetStowedPose extends Command {
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private Claw claw;

    public SetStowedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        claw = Claw.getInstance();

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
        else if(arm.isArmScoringPose() || arm.isPreScorePose() || arm.isDoubleSSPose()){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kScoringPoseMaxVelToStow, ShoulderConstants.kScoringPoseMaxAccelToStow);
        }

        // If we're supposed to monitor the cone's horizontal alignment in the intake, do so before proceeding to stowed
        // Similar with cubes.
        // But if the arm state is already stowed, then we must have gotten stuck / stow has been requested again, so skip it.
        // (Note there is a time limit on monitoring cone/cube alignment to prevent this, but this is here as a failsafe).
        if (claw.isMonitorNewConeIntake() && arm.getState() != ArmState.STOWED) {
            arm.setWristPosition(WristConstants.kMonitorConeAlignmentAngle);
        } else if (claw.isMonitorNewCubeIntake() && arm.getState() != ArmState.STOWED)
            arm.setWristPosition(WristConstants.kMonitorCubeAlignmentAngle);
        else {
            arm.setWristPosition(wrist.getkStowedAngle());
        }

        // Normal speed should be fine to move the arm towards the stowed position.
        // Motion profile to transitory and we'll switch when the shoulder gets lower.
        if(arm.isShoulderAboveAngle(-20)){
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
        }

        // Set various variables to their correct state, or otherwise initialize them.
        arm.setState(ArmState.STOWED);
        arm.setGoalPose(ArmState.NONE);

        // If we're not monitoring alignment, stowing is always a good time to ensure the limelight pipelines are correct.
        // Note that currently, monitoring of cone alignment is the default pipeline anyway for the front limelight.
        if(!claw.isMonitorNewConeIntake() && !claw.isMonitorNewCubeIntake()){
            claw.returnLimelightToDefaultState();
        }

    }

    @Override
    public void execute() {
        // Switch profiles into something slower so we don't smash the arm into the fulcrum.
        if (arm.isShoulderBelowAngle(-20)) {
            arm.setShoulderPositionSmartMotion(shoulder.getkStowedAngle(), SmartMotionArmSpeed.SLOW);
        }
 
        // If we are done monitoring cone/cube alignment, proceed to stowed.
        if (!claw.isMonitorNewConeIntake() && !claw.isMonitorNewCubeIntake()) {
            arm.setWristPosition(wrist.getkStowedAngle());
        }

    }

    @Override
    public void end(boolean interrupted) {
        shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
        ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);

        // We are definitely not monitoring cone/cube intake alignment anymore. Make sure of this.
        claw.setMonitorNewConeIntake(false);
        claw.setMonitorNewCubeIntake(false);
        
        if (!interrupted) {
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(wrist.getkStowedAngle()) && arm.isShoulderAtAngle(shoulder.getkStowedAngle());
    }

}
