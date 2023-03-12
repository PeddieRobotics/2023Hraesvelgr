package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
        BLINK_GREEN_ACQUIRED, BLINK_GREEN_SCORED, BLINK_RED, FLASH_PINK, FLASH_GOLD, FLASH_PURPLE, PULSE_GOLD, PULSE_PURPLE};

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
        set(0.91);
    }

    public void gold(){
        set(0.67);
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

     // Blinks red then briefly goes solid red (for various failure modes)
     public void failure() {
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.BLINK_RED;
    }   

    // Blinks green then briefly goes solid green when you have acquired a game piece
    public void acquiredGamePiece() {
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.BLINK_GREEN_ACQUIRED;
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
    
    // Blinks green twice, then turns the LEDs to solid Green when you have scored a game piece
    public void scoredGamePiece(){
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.BLINK_GREEN_SCORED;
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
    public void homingArm(){
        initialTime = Timer.getFPGATimestamp();
        state = BlinkinState.FLASH_PINK;
    }

    public void emptyCheckForFailure(){
        if(!claw.hasGamepiece() && (state == BlinkinState.GOLD_SOLID || state == BlinkinState.PURPLE_SOLID)){
            failure();
        }
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
                case BLINK_GREEN_SCORED:
                    blinkGreenScoredGamepiece();
                case BLINK_GREEN_ACQUIRED:
                    blinkGreenAcquiredGamepiece();
                case BLINK_RED:
                    blinkRed();
                case FLASH_PINK:
                    flashPink();
                case FLASH_GOLD:
                    flashGold();
                case FLASH_PURPLE:
                    flashPurple();
                case PULSE_GOLD:
                    pulseGold();
                case PULSE_PURPLE:
                    pulsePurple();
                default:
                    black();
            }
        }
    }

    private void blinkRed() {
        red();
        if(currentTime - initialTime > 0.2){
            black();
        } else if(currentTime - initialTime > 0.4){
            red();
        } else if(currentTime - initialTime > 0.6){
            black();
        } else if(currentTime - initialTime > 0.8){
            red();
        } else if(currentTime - initialTime > 1){
            returnToRobotState();
        }
    }

    private void blinkGreenAcquiredGamepiece() {
        green();
        if(currentTime - initialTime > 0.2){
            black();
        } else if(currentTime - initialTime > 0.4){
            green();
        } else if(currentTime - initialTime > 0.6){
            black();
        } else if(currentTime - initialTime > 0.8){
            green();
        } else if(currentTime - initialTime > 1){
            returnToRobotState();
        }
    }

    private void blinkGreenScoredGamepiece() {
        green();
        if(currentTime - initialTime > 0.2){
            black();
        } else if(currentTime - initialTime > 0.4){
            green();
        } else if(currentTime - initialTime > 0.6){
            black();
        } else if(currentTime - initialTime > 0.8){
            green();
        } else if(currentTime - initialTime > 1){
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
