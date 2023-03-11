package frc.robot.commands.ArmCommands;

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
    private boolean transitory;
    private Shoulder shoulder;
    private Wrist wrist;
    private Claw claw;

    public SetStowedPose() {
        arm = Arm.getInstance();
        addRequirements(arm);
        transitory = false;

        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        claw = Claw.getInstance();
    }

    @Override
    public void initialize() {
        transitory = false;
        if (claw.isMonitorNewConeIntake()) {
            arm.setWristPosition(WristConstants.kMonitorConeAlignmentAngle);
        } else {
            arm.setWristPosition(wrist.getkStowedAngle());
        }

        arm.setShoulderPositionSmartMotion(shoulder.getkTransitoryAngle(), SmartMotionArmSpeed.REGULAR);
        arm.setState(ArmState.STOWED);

    }

    @Override
    public void execute() {
        if (arm.isShoulderBelowAngle(-30)) {
            arm.setShoulderPositionSmartMotion(shoulder.getkStowedAngle(), SmartMotionArmSpeed.SLOW);
        }

        if (!claw.isMonitorNewConeIntake()) {
            arm.setWristPosition(wrist.getkStowedAngle());
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            arm.holdShoulderPosition();
        }

    }

    @Override
    public boolean isFinished() {
        return arm.isWristAtAngle(wrist.getkStowedAngle()) && arm.isShoulderAtAngle(shoulder.getkStowedAngle());
    }

}
