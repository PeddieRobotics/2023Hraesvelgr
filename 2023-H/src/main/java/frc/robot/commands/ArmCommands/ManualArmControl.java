package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.OperatorOI;

public class ManualArmControl extends CommandBase{
    private Arm arm;
    private OperatorOI oi;
    private double wristOffset, shoulderOffset;
    private double currentWrist, currentShoulder;

    public ManualArmControl(double wristOffset, double shoulderOffset) {
        arm = Arm.getInstance();
        addRequirements(arm);
        this.wristOffset = wristOffset;
        this.shoulderOffset = shoulderOffset;
    }

    @Override
    public void initialize() {
        currentWrist = arm.getWristPosition();
        currentShoulder = arm.getShoulderPosition();
        oi = OperatorOI.getInstance();
        wristOffset = oi.getWristPIDOffset();
        shoulderOffset = oi.getShoulderPIDOffset();
    }
    
    @Override
    public void execute() {
        wristOffset = oi.getWristPIDOffset();
        shoulderOffset = oi.getShoulderPIDOffset();
        SmartDashboard.putNumber("SHOULDER PID OFFSET", currentShoulder);
        SmartDashboard.putNumber("WRIST PID OFFSET", currentWrist);
        arm.setShoulderPosition(currentShoulder += shoulderOffset);
        arm.setWristPosition(currentWrist += wristOffset);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
