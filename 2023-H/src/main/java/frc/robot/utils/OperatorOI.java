package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.Constants.OIConstants;

public class OperatorOI {
    public enum AlignGoalColumn {
        kCenter, kLeft, kRight
    }

    private static OperatorOI instance;

    public static OperatorOI getInstance() {
        if (instance == null) {
            instance = new OperatorOI();
        }

        return instance;
    }

    private PS4Controller controller;

    /**
     * the center depending on aliance
     */
    private int alignGoalAprilTagID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 2;
    private AlignGoalColumn alignGoalColumn = AlignGoalColumn.kCenter;

    private Arm arm;
    private Blinkin blinkin;
    private Claw claw;
    private Superstructure superstructure;    

    private boolean usePreScorePose;

    private Trigger xButton, touchpadButton, circleButton, triangleButton;

    public OperatorOI() {
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();
        superstructure = Superstructure.getInstance();

        usePreScorePose = OIConstants.kUsePreScorePose;

        configureController(false);
    }

    public int getAlignGoalAprilTagID() {
        return alignGoalAprilTagID;
    }

    public AlignGoalColumn getAlignGoalColumn() {
        return alignGoalColumn;
    }

    public void controlLoop(){
        if(xButton.getAsBoolean()){
            superstructure.requestState(SuperstructureState.SCORE_L1);
        } else if(touchpadButton.getAsBoolean()){
            superstructure.requestState(SuperstructureState.STOWED);
        } else if(circleButton.getAsBoolean()){
            if(claw.isBothSensors()){
                superstructure.requestState(SuperstructureState.CONE_L2);
            } else if (claw.isFrontSensor() && !claw.isBackSensor()){
                superstructure.requestState(SuperstructureState.CUBE_L2);
            }
        } else if(triangleButton.getAsBoolean()){
            if(claw.isBothSensors()){
                superstructure.requestState(SuperstructureState.PRE_CONE_INVERTED_L3);
            } else if (claw.isFrontSensor() && !claw.isBackSensor()){
                superstructure.requestState(SuperstructureState.CUBE_L3);
            }
        }
    }

    public void configureController(boolean usePreScorePose) {
        controller = new PS4Controller(1);

        // Arm Poses
        // L1 score (will move to this pose regardless of having a gamepiece)
        xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);

        // L2 scoring pose
        circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);

        // L3 scoring pose - does not include L3 cone forward right now.
        triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);

        // Square button forces the robot to look at odometry updates.
        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);

        // Stowed pose
        touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);

        // Mute homes the entire arm subsystem, both wrist and shoulder.
        Trigger muteButton = new JoystickButton(controller, 15);

        // Manual Wrist and Shoulder Override Controls
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        // Gyro reset
        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);

        // Press and hold for outtaking slow (gamepiece adjustment), with down arrow this becomes full speed.
        Trigger startButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);

        // Press and hold for intaking slow (gamepiece adjustment), with down arrow this becomes full speed.
        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);

        // Game piece selection / LED indication requests to human player
        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);

        // Column Selection
        Trigger dpadUpTrigger = new Trigger(() -> controller.getPOV() == 0);

        Trigger dpadLeftTrigger = new Trigger(() -> controller.getPOV() == 270);

        Trigger dpadRightTrigger = new Trigger(() -> controller.getPOV() == 90);

        Trigger dpadDownTrigger = new Trigger(() -> controller.getPOV() == 180);

    }

    private boolean bothBumpersHeld() {
        return controller.getL1Button() && controller.getR1Button();
    }

    private boolean bothTriggersHeld() {
        return leftTriggerHeld() & rightTriggerHeld();
    }

    private boolean leftTriggerHeld(){
        return controller.getL2Button();
    }

    private boolean onlyLeftTriggerHeld(){
        return leftTriggerHeld() && !rightTriggerHeld();
    }

    private boolean rightTriggerHeld(){
        return controller.getR2Button();
    }

    private boolean onlyRightTriggerHeld(){
        return !leftTriggerHeld() && rightTriggerHeld();
    }

    private boolean onlyOneTriggerHeld() {
        return leftTriggerHeld() ^ rightTriggerHeld();
    }

    public boolean dPadDownHeld(){
        return controller.getPOV() == 180;
    }

    public double getShoulderPIDOffset() {
        double rawAxis = controller.getRawAxis(PS4Controller.Axis.kRightY.value);
        if (Math.abs(rawAxis) < Constants.OIConstants.kDrivingDeadband) {
            return 0;
        }
        return -1*rawAxis/4;
    }

    public double getWristPIDOffset() {
        double rawAxis = controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        if (Math.abs(rawAxis) < Constants.OIConstants.kDrivingDeadband) {
            return 0;

        }
        return -1*rawAxis;
    }

    public boolean isUsePreScorePose() {
        return usePreScorePose;
    }

    // Only update the boolean for using the pre-score pose if it is a state change
    // This is especially important since this requires configuring the controller mapping
    // for the operator, which should be done infrequently/minimally.
    public void setUsePreScorePose(boolean usePreScorePose) {
        if(this.usePreScorePose != usePreScorePose){
            this.usePreScorePose = usePreScorePose;
            configureController(usePreScorePose);
        }
    }

}