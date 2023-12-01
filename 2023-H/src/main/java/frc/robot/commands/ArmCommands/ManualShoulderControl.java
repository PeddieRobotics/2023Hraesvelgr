package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Constants.ShoulderConstants;

public class ManualShoulderControl extends Command{
    private Arm arm;
    private OperatorOI oi;
    private Blinkin blinkin;

    private double currentShoulder, shoulderOffset;

    public ManualShoulderControl() {
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
    }

    @Override
    public void initialize() {
        currentShoulder = arm.getShoulderPosition();
        oi = OperatorOI.getInstance();

        blinkin.specialOperatorFunctionality();
    }
    
    @Override
    public void execute() {
        shoulderOffset = oi.getShoulderPIDOffset();
        currentShoulder += shoulderOffset;
        if(currentShoulder > ShoulderConstants.kAngleMax){
            currentShoulder = ShoulderConstants.kAngleMax;
        }
        if(currentShoulder < ShoulderConstants.kAngleMin){
            currentShoulder = ShoulderConstants.kAngleMin;
        }
        arm.setShoulderPosition(currentShoulder);
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
