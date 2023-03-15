package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetStowedPose extends CommandBase {
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

        SmartDashboard.putNumber("floorIntakeConeMaxVel", 1250);
        SmartDashboard.putNumber("floorIntakeConeMaxAccel", 4000);

        SmartDashboard.putNumber("floorIntakeCubeMaxVel", 1700);
        SmartDashboard.putNumber("floorIntakeCubeMaxAccel", 4000);

        SmartDashboard.putNumber("scorePoseMaxVel", 1500);
        SmartDashboard.putNumber("scorePoseMaxAccel", 6000);
    }

    @Override
    public void initialize() {
        // Determine the speed of the final stow based on where we are coming from
        if(arm.getState() == ArmState.FLOOR_INTAKE_CONE_EXTENDED){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel,SmartDashboard.getNumber("floorIntakeConeMaxVel", 0), SmartDashboard.getNumber("floorIntakeConeMaxAccel", 0));
        }
        else if(arm.getState() == ArmState.FLOOR_INTAKE_CUBE_EXTENDED){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, SmartDashboard.getNumber("floorIntakeCubeMaxVel", 0), SmartDashboard.getNumber("floorIntakeCubeMaxAccel", 0));          
        }
        else if(arm.isArmScoringPose() || arm.isPreScorePose()){
            shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
            ShoulderConstants.kSmartMotionSlowMinVel, SmartDashboard.getNumber("scorePoseMaxVel", 0), SmartDashboard.getNumber("scorePoseMaxAccel", 0));
        }

        // If we're supposed to monitor the cone's horizontal alignment in the intake, do so before proceeding to stowed
        // But if the arm state is already stowed, then we must have gotten stuck in the monitoring process, so skip it.
        if (claw.isMonitorNewConeIntake() && arm.getState() != ArmState.STOWED) {
            arm.setWristPosition(WristConstants.kMonitorConeAlignmentAngle);
        } else {
            arm.setWristPosition(wrist.getkStowedAngle());
        }

        if(arm.isShoulderAboveAngle(-20)){
            arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
        }

        arm.setState(ArmState.STOWED);
        arm.setGoalPose(ArmState.NONE);

        claw.returnLimelightToDefaultState();
    }

    @Override
    public void execute() {
        if (arm.isShoulderBelowAngle(-20)) {
            arm.setShoulderPositionSmartMotion(shoulder.getkStowedAngle(), SmartMotionArmSpeed.SLOW);
        }

        if (!claw.isMonitorNewConeIntake()) {
            arm.setWristPosition(wrist.getkStowedAngle());
        }
    }

    @Override
    public void end(boolean interrupted) {
        shoulder.setSlowSmartMotionParameters(ShoulderConstants.kSmartMotionSlowSetpointTol,
        ShoulderConstants.kSmartMotionSlowMinVel, ShoulderConstants.kSmartMotionSlowMaxVel, ShoulderConstants.kSmartMotionSlowMaxAccel);

        if (!interrupted) {
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(wrist.getkStowedAngle()) && arm.isShoulderAtAngle(shoulder.getkStowedAngle());
    }

}
