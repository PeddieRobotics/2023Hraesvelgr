package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Constants.WristConstants;

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
        currentWrist += wristOffset;
        if(currentWrist > WristConstants.kAngleMax){
            currentWrist = WristConstants.kAngleMax;
        }
        if(currentWrist < WristConstants.kAngleMin){
            currentWrist = WristConstants.kAngleMin;
        }
        arm.setWristPosition(currentWrist);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
