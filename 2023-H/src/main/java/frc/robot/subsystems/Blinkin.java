package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.BlinkinConstants;

public class Blinkin extends SubsystemBase{
    
    private static Blinkin blinkin;
    private PWMSparkMax blinkinController;

    public Blinkin() {
        blinkinController = new PWMSparkMax(BlinkinConstants.kPwmPort);
        peddieBlue();
    }

    public void peddieBlue() {
        set(0.85);
    }

    public void set(double value) {
        blinkin.set(value);
    }

    public static Blinkin getInstance(){
        if(blinkin == null){
            blinkin = new Blinkin();
        }
        return blinkin;
    }
}
