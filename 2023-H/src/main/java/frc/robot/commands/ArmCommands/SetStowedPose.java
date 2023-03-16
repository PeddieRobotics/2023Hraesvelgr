package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shoulder.SmartMotionArmSpeed;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.ShoulderConstants;
import frc.robot.utils.Constants.WristConstants;

public class SetStowedPose extends CommandBase {
    private Arm arm;
    private Shoulder shoulder;
    private Wrist wrist;
    private Claw claw;

    private boolean arrivedAtMonitorAngle;
    private double monitorGamepieceInitialTime, currentTime;

    public SetStowedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        claw = Claw.getInstance();

        SmartDashboard.putNumber("floorIntakeConeMaxVel", 1200);
        SmartDashboard.putNumber("floorIntakeConeMaxAccel", 2500);

        SmartDashboard.putNumber("floorIntakeCubeMaxVel", 1500);
        SmartDashboard.putNumber("floorIntakeCubeMaxAccel", 5000);

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

        arrivedAtMonitorAngle = false;
        currentTime = Timer.getFPGATimestamp();
        monitorGamepieceInitialTime = Timer.getFPGATimestamp();

        // If we're not monitoring alignment, stowing is always a good time to ensure the limelight pipelines are correct.
        // Note that currently, monitoring of cone alignment is the default pipeline anyway for the front limelight.
        if(!claw.isMonitorNewConeIntake() && !claw.isMonitorNewCubeIntake()){
            claw.returnLimelightToDefaultState();
        }
    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();

        // Switch profiles into something slower so we don't smash the arm into the fulcrum.
        if (arm.isShoulderBelowAngle(-20)) {
            arm.setShoulderPositionSmartMotion(shoulder.getkStowedAngle(), SmartMotionArmSpeed.SLOW);
        }
 
        // If we're monitoring alignment of new cones or cubes in the intake, keep track of when we got there.
        // We need to make sure we don't spend too long looking at this.
        if(claw.isMonitorNewConeIntake()){
            if(arm.isWristAtAngle(WristConstants.kMonitorConeAlignmentAngle) && !arrivedAtMonitorAngle){
                arrivedAtMonitorAngle = true;
                monitorGamepieceInitialTime = Timer.getFPGATimestamp();
            }
        } else if(claw.isMonitorNewCubeIntake()){
            if(arm.isWristAtAngle(WristConstants.kMonitorCubeAlignmentAngle) && !arrivedAtMonitorAngle){
                arrivedAtMonitorAngle = true;
                monitorGamepieceInitialTime = Timer.getFPGATimestamp();
            }
        }

        // Set a time limit how long we can look at the game piece for alignment
        if(arrivedAtMonitorAngle && currentTime - monitorGamepieceInitialTime > ClawConstants.maximumGamepieceMonitorTime){
            claw.setMonitorNewConeIntake(false);
            claw.setMonitorNewCubeIntake(false);
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
