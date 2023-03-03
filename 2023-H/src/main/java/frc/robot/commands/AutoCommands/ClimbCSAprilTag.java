package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class ClimbCSAprilTag extends CommandBase{
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private LimelightBack limelightBack;
    private double speed, robotHeading, startX;
    private boolean climbAwayFromScoringGrid, useLLFront,lostTarget;

    public ClimbCSAprilTag(double speed, double robotHeading, boolean climbAwayFromScoringGrid, boolean useLLFront){
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();
        limelightBack = LimelightBack.getInstance();

        addRequirements(drivetrain);
        this.speed = speed;
        this.robotHeading = robotHeading;
        this.climbAwayFromScoringGrid = climbAwayFromScoringGrid;
        this.useLLFront = useLLFront;

    }

    @Override
    public void initialize() {
        Translation2d chargeStationVector = new Translation2d(speed, new Rotation2d(Math.toRadians(robotHeading)));
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());

        limelightBack.startAveragingX();
        lostTarget=false;
    }

    @Override
    public void execute() {
        if(!limelightBack.hasTarget() && !lostTarget) {
            startX = drivetrain.getPose().getX();
            SmartDashboard.putNumber("Start X", startX);
            lostTarget=true;
        }
        if(lostTarget){
            Translation2d chargeStationVector = new Translation2d(speed/3, new Rotation2d(Math.toRadians(robotHeading)));
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        // Below, red side is untested, as is the entire climb towards grid (else case).
        if(lostTarget){
            double deltaX = Math.abs(drivetrain.getPose().getX()-startX);
            SmartDashboard.putNumber("Delta X", deltaX);
            return(Math.abs(drivetrain.getPose().getX()-startX)> 1);

        }
        if(climbAwayFromScoringGrid){
            if(DriverStation.getAlliance() == Alliance.Blue){
                if(useLLFront){
                    return limelightFront.getBotpose().getX() > 3.75;
                }
                else{
                    return limelightBack.getBotpose().getX() > 3.75;
                }
            }
            else if(DriverStation.getAlliance() == Alliance.Red){
                if(useLLFront){
                    return limelightFront.getBotpose().getX() < 12.7;
                }
                else{
                    return limelightBack.getBotpose().getX() < 12.7;
                }
            }
            return false;
        }
        else{
            if(DriverStation.getAlliance() == Alliance.Blue){
                if(useLLFront){
                    return limelightFront.getBotpose().getX() < 4.0;
                }
                else{
                    return limelightBack.getAveragePoseX() < 4.0;
                }
            }
            else if(DriverStation.getAlliance() == Alliance.Red){
                if(useLLFront){
                    return limelightFront.getBotpose().getX() > 12.9;
                }
                else{
                    return limelightBack.getBotpose().getX() > 12.9;
                }
            }
            return false;
        }
    }

    
}
