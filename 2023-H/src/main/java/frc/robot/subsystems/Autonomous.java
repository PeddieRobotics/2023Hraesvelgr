package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autonomous extends SubsystemBase{
    public static Autonomous instance;

    
    public Autonomous() {

    public static Autonomous getInstance() {
        if (instance == null) {
          instance = new Autonomous();
        }
        return instance;
      }
    }
}
