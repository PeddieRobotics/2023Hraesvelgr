package frc.robot.commands.AutoCommands;

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

public class ClimbCSTilt extends CommandBase{
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private LimelightBack limelightBack;
    private double speed, robotHeading, startX;
    private double initialPitch, currentPitch, pitchRate, pitchThreshold, pitchRateThreshold,
    initialTime, currentTime, climbedTime, minTimeToClimb, minTimeToWaitForTilt;
    private boolean climbAwayFromScoringGrid, useLLFront, climbForward, hasClimbed, hasTilted;

    public ClimbCSTilt(double speed, double robotHeading, boolean climbAwayFromScoringGrid, boolean useLLFront){
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();
        limelightBack = LimelightBack.getInstance();

        addRequirements(drivetrain);
        this.speed = speed;
        this.robotHeading = robotHeading;
        this.climbAwayFromScoringGrid = climbAwayFromScoringGrid;
        this.useLLFront = useLLFront;

        SmartDashboard.putNumber("drivebackDistance", 0.5);
        SmartDashboard.putBoolean("hasClimbed", hasClimbed);
        SmartDashboard.putBoolean("hasTilted", hasTilted);
        SmartDashboard.putNumber("Delta X", 0.0);
        SmartDashboard.putNumber("Pitch threshold", 12);
        SmartDashboard.putNumber("Pitch rate threshold", 15);
        SmartDashboard.putNumber("minTimeToClimb", 2.0);
        SmartDashboard.putNumber("minTimeToWaitForTilt", 1.0);
        SmartDashboard.putBoolean("climbForward", false);


        hasClimbed = false;
        hasTilted = false;
        climbForward = false;
        initialTime = 0.0;
        currentTime = 0.0;
        climbedTime = 0.0;

    }

    @Override
    public void initialize() {
        hasClimbed = false;
        hasTilted = false;
        initialTime = Timer.getFPGATimestamp();
        currentTime = initialTime;

        currentPitch = drivetrain.getPitch();
        initialPitch = currentPitch;

        // Figure out what to look for on tilt based on which way the robot will be climbing
        if(climbAwayFromScoringGrid && useLLFront){
            climbForward = false;
        }
        else if(climbAwayFromScoringGrid && !useLLFront){
            climbForward = true;
        }
        else if(!climbAwayFromScoringGrid && useLLFront){
            climbForward = true;
        }
        else if(!climbAwayFromScoringGrid && !useLLFront){
            climbForward = false;
        }
        pitchThreshold = SmartDashboard.getNumber("Pitch threshold", 10);
        pitchRateThreshold = SmartDashboard.getNumber("Pitch rate threshold", 15);

        if(climbForward){
            pitchRateThreshold *= -1;
        }
        SmartDashboard.putNumber("Computed pitch rate threshold", pitchRateThreshold);
        
        minTimeToClimb = SmartDashboard.getNumber("minTimeToClimb", 2.0);
        minTimeToWaitForTilt = SmartDashboard.getNumber("minTimeToWaitForTilt", 1.0);
        SmartDashboard.putBoolean("climbForward", climbForward);

        Translation2d chargeStationVector = new Translation2d(speed, new Rotation2d(Math.toRadians(robotHeading)));
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());

    }

    @Override
    public void execute() {
        currentTime = Timer.getFPGATimestamp();
        currentPitch = drivetrain.getPitch();
        pitchRate = drivetrain.getAveragePitchRate();
        SmartDashboard.putBoolean("hasClimbed", hasClimbed);
        SmartDashboard.putBoolean("hasTilted", hasTilted);
        SmartDashboard.putBoolean("climbForward", climbForward);
        SmartDashboard.putNumber("Computed pitch rate threshold", pitchRateThreshold);

        if(climbForward){
            if(!hasClimbed && (currentTime-initialTime) > minTimeToClimb && Math.abs(currentPitch-initialPitch) > pitchThreshold){
                hasClimbed = true;
                climbedTime = Timer.getFPGATimestamp();
                Translation2d chargeStationVector = new Translation2d(speed/2, new Rotation2d(Math.toRadians(robotHeading)));
                drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
            }

            if((currentTime-climbedTime) > minTimeToWaitForTilt && hasClimbed){
                if(pitchRate < pitchRateThreshold && !hasTilted) {
                    startX = drivetrain.getPose().getX();
                    hasTilted = true;
                    Translation2d chargeStationVector = new Translation2d(speed/2, new Rotation2d(Math.toRadians(180-robotHeading)));
                    drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
                }
             }
        }
        else{
            if(!hasClimbed && (currentTime-initialTime) > minTimeToClimb && Math.abs(currentPitch-initialPitch) > pitchThreshold){
                hasClimbed = true;
                climbedTime = Timer.getFPGATimestamp();
                Translation2d chargeStationVector = new Translation2d(speed/2, new Rotation2d(Math.toRadians(robotHeading)));
                drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
            }    
            
            if((currentTime-climbedTime) > minTimeToWaitForTilt && hasClimbed){
                if(pitchRate > pitchRateThreshold && !hasTilted) {
                    startX = drivetrain.getPose().getX();
                    hasTilted = true;
                    Translation2d chargeStationVector = new Translation2d(speed/2, new Rotation2d(Math.toRadians(180-robotHeading)));
                    drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
                }
             }
        }


    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        if(hasTilted){
            double deltaX = Math.abs(drivetrain.getPose().getX()-startX);
            SmartDashboard.putNumber("Delta X", deltaX);
            return deltaX > SmartDashboard.getNumber("driveBackDistance", 0.3);
        }
        return false;

    }

    
}
