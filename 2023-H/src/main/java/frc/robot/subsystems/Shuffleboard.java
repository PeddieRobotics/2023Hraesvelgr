package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ShoulderConstants;

public class Shuffleboard extends SubsystemBase{
    public static Shuffleboard shuffleboard;

    private SendableChooser<String> objectiveChooser, gamepieceChooser;

    private Drivetrain drivetrain;
    private Claw claw;
    private Shoulder shoulder;
    private Wrist wrist;

    public Shuffleboard() {
        drivetrain = Drivetrain.getInstance();
        claw = Claw.getInstance();
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();

        setupMiscShuffleboard();
        setupDrivetrainShuffleboard();
        setupClawShuffleboard();
        setupShoulderShuffleboard();
        setupWristShuffleboard();
    }

    public static Shuffleboard getInstance(){
        if(shuffleboard == null){
            shuffleboard = new Shuffleboard();
        }
        return shuffleboard;
    }

    @Override
    public void periodic() {
        updateMiscShuffleboard();
        updateDrivetrainShuffleboard();
        updateClawShuffleboard();
        updateShoulderShuffleboard();
        updateWristShuffleboard();
    }

    private void setupMiscShuffleboard(){
         // Choosers below are for testing purposes, should be removed when full logic /sensors are available
        objectiveChooser = new SendableChooser<String>();
        objectiveChooser.addOption("Cone", "Cone");
        objectiveChooser.addOption("Cube", "Cube");
        objectiveChooser.addOption("None", "None");

        gamepieceChooser = new SendableChooser<String>();
        gamepieceChooser.addOption("Cone", "Cone");
        gamepieceChooser.addOption("Cube", "Cube");
        gamepieceChooser.addOption("None", "None");

    }

    private void updateMiscShuffleboard(){
    }

    private void setupDrivetrainShuffleboard(){
        SmartDashboard.putNumber("Odometry X", drivetrain.getPose().getX());
        SmartDashboard.putNumber("Odometry Y", drivetrain.getPose().getY());
        SmartDashboard.putNumber("Odometry theta", drivetrain.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Heading", drivetrain.getHeading());
        SmartDashboard.putNumber("Snap To Angle Heading", 0);
        SmartDashboard.putBoolean("Use heading correction?", false);

    }

    private void updateDrivetrainShuffleboard(){
        SmartDashboard.putNumber("Odometry X", drivetrain.getPose().getX());
        SmartDashboard.putNumber("Odometry Y", drivetrain.getPose().getY());
        SmartDashboard.putNumber("Odometry theta", drivetrain.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Heading", drivetrain.getHeading());
        SmartDashboard.putNumber("Snap To Angle Heading", 0);
    }

    private void setupClawShuffleboard(){
        SmartDashboard.putNumber("OR: Claw speed", 0.0);
        SmartDashboard.putData("Game Piece", gamepieceChooser);
    }

    private void updateClawShuffleboard(){
        SmartDashboard.putNumber("Claw speed", claw.getClawSpeed());
        SmartDashboard.putNumber("Claw current", claw.getCurrent());
    }

    private void setupShoulderShuffleboard(){
        //dynamic FF parameters
        SmartDashboard.putNumber("Shoulder kS", Constants.ShoulderConstants.kSVolts);
        SmartDashboard.putNumber("Shoulder kG", Constants.ShoulderConstants.kGVolts);
        SmartDashboard.putNumber("Shoulder kV", Constants.ShoulderConstants.kVVoltSecondPerRad);
        SmartDashboard.putNumber("Shoulder kA", Constants.ShoulderConstants.kAVoltSecondSquaredPerRad);

        // PID controller parameters
        SmartDashboard.putNumber("Shoulder P", Constants.ShoulderConstants.kP);
        SmartDashboard.putNumber("Shoulder I", Constants.ShoulderConstants.kI);
        SmartDashboard.putNumber("Shoulder IZone", Constants.ShoulderConstants.kIz);
        SmartDashboard.putNumber("Shoulder D", Constants.ShoulderConstants.kD);
        SmartDashboard.putNumber("Shoulder FF", Constants.ShoulderConstants.kFF);

        // Toggle shoulder pid
        SmartDashboard.putBoolean("Toggle shoulder PID tuning mode", false);

        // Toggle open loop shoulder control
        SmartDashboard.putBoolean("Toggle open loop shoulder control", false);

        // Setpoints for test mode
        SmartDashboard.putNumber("Shoulder speed % setpoint", 0.0);
        SmartDashboard.putNumber("Shoulder PID setpoint (deg)", 0.0);

        // Execute button
        SmartDashboard.putBoolean("Execute", false);
        
        
    }

    private void updateShoulderShuffleboard(){
        // Auxiliary information
        SmartDashboard.putNumber("Shoulder current", shoulder.getOutputCurrent());
        SmartDashboard.putNumber("Shoulder temperature", shoulder.getMotorTemperature());
        SmartDashboard.putNumber("Shoulder encoder pos", shoulder.getPosition());
        SmartDashboard.putNumber("Shoulder % output", shoulder.getSpeed());
        SmartDashboard.putNumber("Shoulder angle", shoulder.getAngle());
        SmartDashboard.putNumber("Shoulder velocity/100", shoulder.getVelocity()/100.0);
        SmartDashboard.putNumber("Shoulder Arbitrary FF", shoulder.getArbitraryFF());
    }
    
    private void setupWristShuffleboard(){
        //dynamic FF parameters
        SmartDashboard.putNumber("Wrist kS", Constants.WristConstants.kSVolts);
        SmartDashboard.putNumber("Wrist kG", Constants.WristConstants.kGVolts);
        SmartDashboard.putNumber("Wrist kV", Constants.WristConstants.kVVoltSecondPerRad);
        SmartDashboard.putNumber("Wrist kA", Constants.WristConstants.kAVoltSecondSquaredPerRad);

        SmartDashboard.putNumber("Wrist Arbitrary FF", wrist.getDynamicFeedForward());
        
        // PID controller parameters
        SmartDashboard.putNumber("Wrist P", Constants.WristConstants.kP);
        SmartDashboard.putNumber("Wrist I", Constants.WristConstants.kI);
        SmartDashboard.putNumber("Wrist D", Constants.WristConstants.kD);
        SmartDashboard.putNumber("Wrist FF", Constants.WristConstants.kFF);

        //Toggle wrist pid
        SmartDashboard.putBoolean("Toggle wrist PID tuning mode", false);

        //Toggle open loop wrist control
        SmartDashboard.putBoolean("Toggle open loop wrist control", false);

        //setpoints
        SmartDashboard.putNumber("Wrist speed % setpoint", 0.0);
        SmartDashboard.putNumber("Wrist PID setpoint (deg)", 0.0);

    }

    private void updateWristShuffleboard(){
        SmartDashboard.putNumber("Wrist encoder pos", wrist.getPosition());
        SmartDashboard.putNumber("Wrist velocity", wrist.getVelocity());
        SmartDashboard.putNumber("Wrist current", wrist.getOutputCurrent());
        SmartDashboard.putNumber("Wrist temperature", wrist.getMotorTemperature());
    }

    // Choosers below are for testing purposes, should be removed when full logic /sensors are available
    public boolean isCurrentObjectiveCone(){
        return objectiveChooser.getSelected().equals("Cone");
    }

    public boolean hasCone(){
        return gamepieceChooser.getSelected().equals("Cone");
    }

    public boolean hasCube(){
        return gamepieceChooser.getSelected().equals("Cube");
    }
}
