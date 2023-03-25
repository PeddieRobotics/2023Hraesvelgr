package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.Constants.BlinkinConstants;
import frc.robot.utils.Constants.GlobalConstants;

public class Blinkin extends SubsystemBase{
    
    private static Blinkin blinkin;
    private Spark blinkinController;
    private double initialTime, currentTime, flashTime;
    private boolean flashOn;
    
    public enum BlinkinState {NONE, GREEN_SOLID, RED_SOLID, GOLD_SOLID, PURPLE_SOLID, PINK_SOLID, AQUA_SOLID,
        BLINK_GREEN, BLINK_RED, FLASH_PINK, FLASH_GOLD, FLASH_PURPLE, PULSE_GOLD, PULSE_PURPLE, STROBE_GOLD, STROBE_PURPLE,
        GYRO_SUCCESS, GYRO_OVERRUN};

    private BlinkinState state;

    private Claw claw;

    public Blinkin() {
        if(GlobalConstants.kUseLEDLights){
            blinkinController = new Spark(BlinkinConstants.kPwmPort);
        }
        claw = Claw.getInstance();

        black();

        state = BlinkinState.NONE;
        flashOn = true; // default to true so the flash happens immediately whenever we run a flashing command

        initialTime = Timer.getFPGATimestamp();
        currentTime = Timer.getFPGATimestamp();
        flashTime = Timer.getFPGATimestamp();
    }

    public BlinkinState getState() {
        return state;
    }

    public static Blinkin getInstance(){
        if(blinkin == null){
            blinkin = new Blinkin();
        }
        return blinkin;
    }

    public void set(double value) {
        if(GlobalConstants.kUseLEDLights){
            blinkinController.set(value);
        }
    }

    // Neutral color to use when doing nothing (LEDs off /black)
    public void black() {
        set(0.99);
    }

    public void purple(){
        set(0.89);
    }

    public void gold(){
        set(0.63);
    }

    public void pink(){
        set(0.57);
    }

    // Solid green color
    public void green(){
        set(0.71);
    }

    // Solid red color
    public void red() {
        set(0.61);
    }

    // Solid red color
    public void aqua() {
        set(0.81);
    }

    // Strobe gold (Color 1 pattern)
    public void strobeGold() {
        if(currentTime - initialTime < 1.0){
            set(0.15);
        }
        else{
            returnToRobotState();
        }
    }

    // Strobe purple (Color 2 pattern)
    public void strobePurple() {
        if(currentTime - initialTime < 1.0){
            set(0.35);
        }
        else{
            returnToRobotState();
        }
    }

    // Blinks red (for various failure modes)
    public void none() {
        state = BlinkinState.NONE;
    }   

    // Blinks green then briefly goes solid green (for various success modes)
    public void success(){
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.BLINK_GREEN;
    }

     // Blinks red then briefly goes solid red (for various failure modes)
     public void failure() {
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.BLINK_RED;
    }   

    // Blinks red then briefly goes solid red (for various failure modes)
    public void gamepieceAnalyzedSuccess() {
        initialTime = Timer.getFPGATimestamp();

        ClawState clawState = claw.getState();
        if(clawState == ClawState.CONE){
            state = BlinkinState.STROBE_GOLD;
        }
        else if(clawState == ClawState.CUBE){
            state = BlinkinState.STROBE_PURPLE;
        }
    }   

    // Blinks the appropriate color when trying to auto-target
    public void acquiringTarget() {
        initialTime = Timer.getFPGATimestamp();

        // Robot has a gamepiece and is trying to score at the goal
        if(claw.hasCone()){
            state = BlinkinState.PULSE_GOLD;
        }
        else if(claw.hasCube()){
            state = BlinkinState.PULSE_PURPLE;
        }
        // Robot has no gamepiece and is trying to auto-target at a human player station
        else{
            // If the robot is currently seeking a cone or cube, slow down to a pulse while targeting at the HP stations
            if(state == BlinkinState.FLASH_GOLD){
                state = BlinkinState.PULSE_GOLD;
            }
            else if(state == BlinkinState.FLASH_PURPLE){
                state = BlinkinState.PULSE_PURPLE;
            }
            // If we've reached this case, then auto-target has failed / does not apply
            else{
                state = BlinkinState.BLINK_RED;
            }
        }
    }

    // Turns the LEDS to flashing purple when intaking a cube
    public void intakingCube() {
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.FLASH_PURPLE;
    }

    // Turns the LEDS to flashing gold when intaking a cone
    public void intakingCone() {
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.FLASH_GOLD;
    }

    // If the arm is being rehomed, flash pink during process (special status)
    // Also used for some other special operator overrides.
    public void specialOperatorFunctionality(){
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.FLASH_PINK;
    }

    public void emptyCheckForFailure(){
        if(state == BlinkinState.GOLD_SOLID || state == BlinkinState.PURPLE_SOLID){
            failure();
            claw.stopClaw();
        }
    }

    public void lockedWheels(){
        state = BlinkinState.AQUA_SOLID;
    }

    public void rainbowTwinkle(){
        if(currentTime - initialTime < 2.0){
            set(-0.55);
        }
        else{
            returnToRobotState();
        }
    }
    
    public void whiteOverride(){
        if(currentTime - initialTime < 2.0){
            set(0.93);
        }
        else{
            returnToRobotState();
        }
    }

    public void gyroClimbSuccess(){
        state = BlinkinState.GYRO_SUCCESS;
    }

    public void gyroClimbOverrun(){
        state = BlinkinState.GYRO_OVERRUN;
    }

    public void returnToRobotState(){
        if(claw.getState() == ClawState.INTAKING_CONE){
            state = BlinkinState.FLASH_GOLD;
        }
        else if(claw.getState() == ClawState.INTAKING_CUBE){
            state = BlinkinState.FLASH_PURPLE;
        }
        else if(claw.getState() == ClawState.CONE){
            state = BlinkinState.GOLD_SOLID;
        }
        else if(claw.getState() == ClawState.CUBE){
            state = BlinkinState.PURPLE_SOLID;
        }     
        else{
            state = BlinkinState.NONE;
        }
    }

    @Override
    public void periodic() {
        if(GlobalConstants.kUseLEDLights){
            currentTime = Timer.getFPGATimestamp();
            switch(state){
                case NONE:
                    black();
                    break;
                case GREEN_SOLID:
                    green();
                    break;
                case RED_SOLID:
                    red();
                    break;
                case GOLD_SOLID:
                    gold();
                    break;
                case PURPLE_SOLID:
                    purple();
                    break;
                case PINK_SOLID:
                    pink();
                    break;
                case AQUA_SOLID:
                    aqua();
                    break;
                case BLINK_GREEN:
                    blinkGreen();
                    break;
                case BLINK_RED:
                    blinkRed();
                    break;
                case FLASH_PINK:
                    flashPink();
                    break;
                case FLASH_GOLD:
                    flashGold();
                    break;
                case FLASH_PURPLE:
                    flashPurple();
                    break;
                case PULSE_GOLD:
                    pulseGold();
                    break;
                case PULSE_PURPLE:
                    pulsePurple();
                    break;
                case STROBE_GOLD:
                    strobeGold();
                    break;
                case STROBE_PURPLE:
                    strobePurple();
                    break;
                case GYRO_SUCCESS:
                    gyroClimbSuccess();
                    break;
                case GYRO_OVERRUN:
                    gyroClimbOverrun();
                    break;
                default:
                    black();
            }
        }
    }

    private void blinkRed() {
        if(currentTime - initialTime < 0.15){
            red();
        } else if(currentTime - initialTime < 0.3){
            black();
        } else if(currentTime - initialTime < 0.45){
            red();
        } else if(currentTime - initialTime < 0.6){
            black();
        } else if(currentTime - initialTime < 0.75){
            red();
        } else if(currentTime - initialTime < 0.9){
            black();
        } else if(currentTime - initialTime < 1.05){
            red();
        } else if(currentTime - initialTime < 1.2){
            black();
        } else if(currentTime - initialTime < 2.2){
            red();
        } 
        else{
            returnToRobotState();  
        }
    }

    private void blinkGreen() {
        if(currentTime - initialTime < 0.5){
            green();
        } else if(currentTime - initialTime < 0.7){
            black();
        } else if(currentTime - initialTime < 1.2){
            green();
        } else if(currentTime - initialTime < 1.4){
            black();
        } else if(currentTime - initialTime < 2.5){
            green();
        } else{
            returnToRobotState();  
        }
    }

    // TODO: Reconsider with 5v Blinkin options
    private void flashPink() {
        if(flashOn){
            pink();
            if(currentTime - flashTime > 0.3){
                flashOn = false;
                flashTime = Timer.getFPGATimestamp();
            }

        }
        else{
            black();
            if(currentTime - flashTime > 0.3){
                flashOn = true;
                flashTime = Timer.getFPGATimestamp();
            }
        }
    }

    // TODO: Reconsider with 5v Blinkin options
    private void flashGold() {
        if(flashOn){
            gold();
            if(currentTime - flashTime > 0.3){
                flashOn = false;
                flashTime = Timer.getFPGATimestamp();
            }

        }
        else{
            black();
            if(currentTime - flashTime > 0.3){
                flashOn = true;
                flashTime = Timer.getFPGATimestamp();
            }
        }
    }

    // TODO: Reconsider with 5v Blinkin options
    private void flashPurple() {
        if(flashOn){
            purple();
            if(currentTime - flashTime > 0.3){
                flashOn = false;
                flashTime = Timer.getFPGATimestamp();
            }

        }
        else{
            black();
            if(currentTime - flashTime > 0.3){
                flashOn = true;
                flashTime = Timer.getFPGATimestamp();
            }
        }
    }

    // TODO: Need 5v Blinkin to do pulse pattern
    private void pulseGold() {
        flashGold();
    }

    // TODO: Need 5v Blinkin to do pulse pattern
    private void pulsePurple() {
        flashPurple();
    }

}
