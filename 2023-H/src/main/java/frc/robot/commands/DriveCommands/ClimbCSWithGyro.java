package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.Constants.AutoConstants;

public class ClimbCSWithGyro extends CommandBase{
    private Drivetrain drivetrain;
    
    private double speed, robotHeading, currentPitch, initialPitch, pitchThresholdOnCS, pitchDifference;

    private boolean climbAwayFromScoringGrid, droveOntoCS, hasTilted;

    public ClimbCSWithGyro(double speed, double robotHeading, boolean climbAwayFromScoringGrid){
        drivetrain = Drivetrain.getInstance();

        addRequirements(drivetrain);

        this.speed = speed;
        this.robotHeading = robotHeading;
        this.climbAwayFromScoringGrid = climbAwayFromScoringGrid;

        pitchThresholdOnCS = SmartDashboard.getNumber("Pitch threshold on CS", AutoConstants.kCSAngleOnCSDegrees);

        SmartDashboard.putBoolean("droveOntoCS", false);
        SmartDashboard.putBoolean("hasTilted", false);
    }

    @Override
    public void initialize() {
        currentPitch = drivetrain.getPitch();
        initialPitch = currentPitch;

        pitchThresholdOnCS = SmartDashboard.getNumber("Pitch threshold on CS", AutoConstants.kCSAngleOnCSDegrees);

        // Start driving onto the charge station in the correct direction
        Translation2d chargeStationVector = new Translation2d(speed, new Rotation2d(Math.toRadians(robotHeading)));
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());

    }

    @Override
    public void execute() {
        currentPitch = drivetrain.getPitch();

        SmartDashboard.putBoolean("droveOntoCS", droveOntoCS);
        SmartDashboard.putBoolean("hasTilted", hasTilted);

        if(Math.abs(currentPitch-initialPitch) > pitchThresholdOnCS){
            droveOntoCS = true;
            pitchDifference = Math.abs(currentPitch-initialPitch);
            Translation2d chargeStationVector = new Translation2d(speed/3, new Rotation2d(Math.toRadians(robotHeading)));
            drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
        }



    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
        drivetrain.lock();
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.getMatchTime() > 14.5){
            return true;
        }
        if(hasTilted){
            return true;
        }
        return false;
    }

    
}