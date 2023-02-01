package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase{
    
    private static PWMSparkMax blinkin;

    public Blinkin(int pwmPort) {
        blinkin = new PWMSparkMax(pwmPort);
        peddieBlue();
    }

    public void peddieBlue() {
        set(0.85);
    }

    public void set(double value) {
        blinkin.set(value);
    }
}
