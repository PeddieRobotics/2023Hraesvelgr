package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Constants.WristConstants;

public class ManualWristControl extends Command{
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
        if(currentWrist > WristConstants.kL2CubeAngle){
            currentWrist = WristConstants.kL2CubeAngle;
        }
        if(currentWrist < WristConstants.kHomeAngle){
            currentWrist = WristConstants.kHomeAngle;
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
