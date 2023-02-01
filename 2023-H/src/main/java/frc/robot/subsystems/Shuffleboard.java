package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shuffleboard extends SubsystemBase{
    public static Shuffleboard shuffleboard;
    private Drivetrain drivetrain;
    private Claw claw;


    public Shuffleboard(){
        drivetrain = Drivetrain.getInstance();
        claw = Claw.getInstance();

        drivetrainShuffleboard();
        clawShuffleboard();
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
}
