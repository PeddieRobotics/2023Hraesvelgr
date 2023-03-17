package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Constants.WristConstants;

public class ManualWristControl extends CommandBase{
    private Arm arm;
    private OperatorOI oi;
    private Blinkin blinkin;

    private double currentWrist, wristOffset;

    public ManualWristControl() {
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();

    }

    @Override
    public void initialize() {
        currentWrist = arm.getWristPosition();

        oi = OperatorOI.getInstance();

        blinkin.specialOperatorFunctionality();
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
        blinkin.returnToRobotState();

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
