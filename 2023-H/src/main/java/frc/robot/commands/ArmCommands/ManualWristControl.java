package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.OperatorOI;

public class ManualWristControl extends CommandBase{
    private Arm arm;
    private OperatorOI oi;
    private double currentWrist, wristOffset;

    public ManualWristControl() {
        arm = Arm.getInstance();
        // addRequirements(arm);
    }

    @Override
    public void initialize() {
        currentWrist = arm.getWristPosition();
        oi = OperatorOI.getInstance();
    }
    
    @Override
    public void execute() {
        wristOffset = oi.getWristPIDOffset();
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
