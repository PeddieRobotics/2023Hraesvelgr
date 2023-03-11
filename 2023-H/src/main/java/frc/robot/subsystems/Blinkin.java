package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.BlinkinConstants;

public class Blinkin extends SubsystemBase{
    
    private static Blinkin blinkin;
    private Spark blinkinController;
    private double initialTime, currentTime;
    private boolean acquiredGamePiece;


    public Blinkin() {
        blinkinController = new Spark(BlinkinConstants.kPwmPort);
        neutral();
    }

    public static Blinkin getInstance(){
        if(blinkin == null){
            blinkin = new Blinkin();
        }
        return blinkin;
    }

    //self-explanatory
    public void neutral() {
        set(0.93);
    }

    public void autoColor() {
        set(-0.99);
    }

    //Turns the LEDS to solid Lawn Green when acquired Game Piece
    public void acquiredGamePiece() {
        set(0.71);
        initialTime = Timer.getFPGATimestamp();
        acquiredGamePiece = true;
    }

    //Turns the LEDS to solid Red when you have no target
    public void noTarget() {
        set(0.61);
    }

    //Turns the LEDS to solid Purple when you are running floor cube intake
    public void intakeCube() {
        set(0.91);
    }

    //Turns the LEDS to yellow when intaking a cone
    public void intakeCone() {
        set(0.69);
    }

    //Aligning to the target from far away
    public void seekingTargetSlow() {
        set(0.03);
    }

    //Aligning to the target from medium
    public void seekingTargetMedium() {
        set(0.05);
    }

    //Aligning to the target from very close
    public void seekingTargetFast() {
        set(0.07);
    }

    //Pattern intensity of Green when we are at the target
    public void atTarget() {
        set(0.43);
    }

    public void set(double value) {
        blinkinController.set(value);
    }

    @Override
    public void periodic() {
        currentTime = Timer.getFPGATimestamp();
        if(acquiredGamePiece && (currentTime - initialTime > 3)){
            blinkin.neutral();
            acquiredGamePiece = false;
        }
    }
}
