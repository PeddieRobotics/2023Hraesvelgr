package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.Hashtable;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Autonomous.Autonomous;
import frc.robot.utils.Constants;

public class Shuffleboard extends SubsystemBase{
    public static Shuffleboard shuffleboard;
    private Autonomous autonomous;
    private Drivetrain drivetrain;
    private Claw claw;
    private Shoulder shoulder;
    private Wrist wrist;

    private SendableChooser<String> startingGamePieceSelector, startingColumnSelector, scoringLevelSelector, objectiveOneSelector, objectiveTwoSelector, objectiveThreeSelector, objectiveFourSelector;

    public static final String Cone = "Cone";
    public static final String Cube = "Cube";

    public static final String ColumnOne = "Column One";
    public static final String ColumnTwo = "Column Two";
    public static final String ColumnThree = "Column Three";
    public static final String ColumnFour = "Column Four";
    public static final String ColumnFive = "Column Five";
    public static final String ColumnSix = "Column Six";
    public static final String ColumnSeven = "Column Seven";
    public static final String ColumnEight = "Column Eight";
    public static final String ColumnNine = "Column Nine";

    public static final String ScoringLevelOne = "Scoring Level One";
    public static final String ScoringLevelTwo = "Scoring Level Two";
    public static final String ScoringLevelThree = "Scoring Level Three";

    public static final String LeaveCommunity = "Leave Community";
    public static final String Balance = "Balance";
    public static final String GrabOne = "Grab One";
    public static final String ScoreOne = "Score One";
    public static final String Prepare = "Prepare";

    public static final String None = "Null";

    public static boolean Inversion = true;




    public Shuffleboard() {
        startingGamePieceSelector = new SendableChooser<String>();
        startingColumnSelector = new SendableChooser<String>();
        scoringLevelSelector = new SendableChooser<String>();
        objectiveOneSelector = new SendableChooser<String>();
        objectiveTwoSelector = new SendableChooser<String>();
        objectiveThreeSelector = new SendableChooser<String>();
        objectiveFourSelector = new SendableChooser<String>();


        drivetrain = Drivetrain.getInstance();
        claw = Claw.getInstance();
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();
        autonomous = autonomous.getInstance();

        drivetrainShuffleboard();
        clawShuffleboard();
        shoulderShuffleboard();
        wristShuffleboard();
        autonomousShuffleboard();
    }

    public static Shuffleboard getInstance(){
        if(shuffleboard == null){
            shuffleboard = new Shuffleboard();
        }
        return shuffleboard;
    }

    @Override
    public void periodic() {
        drivetrainShuffleboard();
        clawShuffleboard();
        shoulderShuffleboard();
        wristShuffleboard();
        autonomousShuffleboard();
    }

    public String returnStartingGamePieceCommand() {
        return startingGamePieceSelector.getSelected();
    }

    public String returnStartingColumnCommand() {
        return startingColumnSelector.getSelected();
    }

    public String returnScoringLevelCommand() {
        return startingGamePieceSelector.getSelected();
    }

    public boolean returnIsInvertedCommand() {
        return Inversion;
    }
    public String returnObjectiveOneCommand() {
        return objectiveOneSelector.getSelected();
    }

    public String returnObjectiveTwoCommand() {
        return objectiveTwoSelector.getSelected();
    }

    public String returnObjectiveThreeCommand() {
        return objectiveThreeSelector.getSelected();
    }

    public String returnObjectiveFourCommand() {
        return objectiveFourSelector.getSelected();
    }

    private void drivetrainShuffleboard(){
        SmartDashboard.putNumber("Odometry X", drivetrain.getPose().getX());
        SmartDashboard.putNumber("Odometry Y", drivetrain.getPose().getY());
        SmartDashboard.putNumber("Heading", drivetrain.getHeading());
        SmartDashboard.putNumber("Snap To Angle Heading", 0);
    }

    private void clawShuffleboard(){
        SmartDashboard.putNumber("OR: Claw speed", 0.0);
        SmartDashboard.putNumber("Claw speed", claw.getClawSpeed());
        SmartDashboard.putNumber("Claw Current", claw.getCurrent());
    }

    private void shoulderShuffleboard(){
        //dynamic FF parameters
        SmartDashboard.putNumber("shoulder kS", Constants.ShoulderConstants.kSVolts);
        SmartDashboard.putNumber("shoulder kG", Constants.ShoulderConstants.kGVolts);
        SmartDashboard.putNumber("shoulder kV", Constants.ShoulderConstants.kVVoltSecondPerRad);
        SmartDashboard.putNumber("shoulder kA", Constants.ShoulderConstants.kAVoltSecondSquaredPerRad);

        SmartDashboard.putNumber("shoulder Arbitrary FF", shoulder.getDynamicFeedForward());

        // PID controller parameters
        SmartDashboard.putNumber("shoulder P", Constants.ShoulderConstants.kP);
        SmartDashboard.putNumber("shoulder I", Constants.ShoulderConstants.kI);
        SmartDashboard.putNumber("shoulder D", Constants.ShoulderConstants.kD);
        SmartDashboard.putNumber("shoulder FF", Constants.ShoulderConstants.kFF);

        //Toggle shoulder pid
        SmartDashboard.putBoolean("shoulder toggle pid active", false);

        //setpoints
        SmartDashboard.putNumber("shoulder speed % setpoint", 0.0);
        SmartDashboard.putNumber("shoulder angle setpoint", 0.0);
        SmartDashboard.putNumber("shoulder velocity setpoint", 0.0);
        
        SmartDashboard.putNumber("shoulder Motor Current", shoulder.getOutputCurrent());
        SmartDashboard.putNumber("shoulder Motor Temperature", shoulder.getMotorTemperature());
        SmartDashboard.putNumber("shoulder encoder", shoulder.getPosition());
        SmartDashboard.putNumber("shoulder angle", shoulder.getAngle());
        SmartDashboard.putNumber("shoulder velocity", shoulder.getVelocity());
    }
    
    private void wristShuffleboard(){
        //dynamic FF parameters
        SmartDashboard.putNumber("wrist kS", Constants.WristConstants.kSVolts);
        SmartDashboard.putNumber("wrist kG", Constants.WristConstants.kGVolts);
        SmartDashboard.putNumber("wrist kV", Constants.WristConstants.kVVoltSecondPerRad);
        SmartDashboard.putNumber("wrist kA", Constants.WristConstants.kAVoltSecondSquaredPerRad);

        SmartDashboard.putNumber("wrist Arbitrary FF", wrist.getDynamicFeedForward());
        
        // PID controller parameters
        SmartDashboard.putNumber("wrist P", Constants.WristConstants.kP);
        SmartDashboard.putNumber("wrist I", Constants.WristConstants.kI);
        SmartDashboard.putNumber("wrist D", Constants.WristConstants.kD);
        SmartDashboard.putNumber("wrist FF", Constants.WristConstants.kFF);

        //Toggle wrist pid
        SmartDashboard.putBoolean("wrist toggle pid active", false);

        //setpoints
        SmartDashboard.putNumber("wrist speed % setpoint", 0.0);
        SmartDashboard.putNumber("wrist angle setpoint", 0.0);
        SmartDashboard.putNumber("wrist velocity setpoint", 0.0);

        SmartDashboard.putNumber("wrist encoder", wrist.getPosition());
        SmartDashboard.putNumber("wrist velocity", wrist.getVelocity());
        SmartDashboard.putNumber("wrist Motor Current", wrist.getOutputCurrent());
        SmartDashboard.putNumber("wrist Motor temperature", wrist.getMotorTemperature());
    }

    private void autonomousShuffleboard() {
        startingGamePieceSelector.setDefaultOption("Cone", Cone);
        startingGamePieceSelector.addOption("Cube", Cube);
        SmartDashboard.putData("Starting Game Piece", startingGamePieceSelector);

        startingColumnSelector.setDefaultOption("Column 1", ColumnOne);
        SmartDashboard.putData("Starting Column", startingColumnSelector);

        scoringLevelSelector.setDefaultOption("Level 1", ScoringLevelOne);
        SmartDashboard.putData("Scoring level", scoringLevelSelector);

        SmartDashboard.putBoolean("Is Inverted", Inversion);

        objectiveOneSelector.setDefaultOption("Nothing", None);
        SmartDashboard.putData("Objective One", objectiveOneSelector);

        objectiveTwoSelector.setDefaultOption("Nothing", None);
        SmartDashboard.putData("Objective Two", objectiveTwoSelector);

        objectiveThreeSelector.setDefaultOption("Nothing", None);
        SmartDashboard.putData("Objective Three", objectiveThreeSelector);

        objectiveFourSelector.setDefaultOption("Nothing", None);
        SmartDashboard.putData("Objective Four", objectiveFourSelector);
    }
}
