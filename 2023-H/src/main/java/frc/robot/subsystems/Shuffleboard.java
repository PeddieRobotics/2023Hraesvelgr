package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Shuffleboard extends SubsystemBase{
    public static Shuffleboard shuffleboard;
    private Drivetrain drivetrain;
    private Claw claw;
    private Shoulder shoulder;
    private Wrist wrist;


    public Shuffleboard(){
        drivetrain = Drivetrain.getInstance();
        claw = Claw.getInstance();
        shoulder = Shoulder.getInstance();
        wrist = Wrist.getInstance();

        drivetrainShuffleboard();
        clawShuffleboard();
        shoulderShuffleboard();
        wristShuffleboard();
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

        // PID controller parameters
        SmartDashboard.putNumber("shoulder P", Constants.ShoulderConstants.kP);
        SmartDashboard.putNumber("shoulder I", Constants.ShoulderConstants.kI);
        SmartDashboard.putNumber("shoulder D", Constants.ShoulderConstants.kD);
        SmartDashboard.putNumber("shoulder FF", Constants.ShoulderConstants.kFF);

        SmartDashboard.putBoolean("shoulder toggle pid active", false);

        SmartDashboard.putNumber("shoulder speed % setpoint", 0.0);
        SmartDashboard.putNumber("shoulder angle setpoint", 0.0);
        SmartDashboard.putNumber("shoulder velocity setpoint", 0.0);
        
        SmartDashboard.putNumber("shoulder Motor Current", shoulder.getOutputCurrent());
        SmartDashboard.putNumber("shoulder Motor Temperature", shoulder.getMotorTemperature());
        SmartDashboard.putNumber("shoulder encoder", shoulder.getPosition());
        SmartDashboard.putNumber("shoulder angle", shoulder.getAngle());
        SmartDashboard.putNumber("shoulder velocity", shoulder.getVelocity());
        
        SmartDashboard.putBoolean("toggle shoulder pid active", false);
    }
    
    private void wristShuffleboard(){
        //dynamic FF parameters
        SmartDashboard.putNumber("wrist kS", Constants.WristConstants.kSVolts);
        SmartDashboard.putNumber("wrist kG", Constants.WristConstants.kGVolts);
        SmartDashboard.putNumber("wrist kV", Constants.WristConstants.kVVoltSecondPerRad);
        SmartDashboard.putNumber("wrist kA", Constants.WristConstants.kAVoltSecondSquaredPerRad);
    
        // PID controller parameters
        SmartDashboard.putNumber("wrist P", Constants.WristConstants.kP);
        SmartDashboard.putNumber("wrist I", Constants.WristConstants.kI);
        SmartDashboard.putNumber("wrist D", Constants.WristConstants.kD);
        SmartDashboard.putNumber("wrist FF", Constants.WristConstants.kFF);
    
        SmartDashboard.putBoolean("wrist toggle pid active", false);
    
        SmartDashboard.putNumber("wrist speed % setpoint", 0.0);
        SmartDashboard.putNumber("wrist angle setpoint", 0.0);
        SmartDashboard.putNumber("wrist velocity setpoint", 0.0);

        SmartDashboard.putNumber("wrist encoder", wrist.getPosition());
        SmartDashboard.putNumber("wrist velocity", wrist.getVelocity());
        SmartDashboard.putNumber("wrist Motor Current", wrist.getOutputCurrent());
        SmartDashboard.putNumber("wrist Motor temperature", wrist.getMotorTemperature());

        SmartDashboard.putBoolean("toggle wrist pid active", false);
    }
}
