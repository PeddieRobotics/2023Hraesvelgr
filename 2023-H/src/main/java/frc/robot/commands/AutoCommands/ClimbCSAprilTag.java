
package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;

public class ClimbCSAprilTag extends CommandBase{
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private LimelightBack limelightBack;
    private double speed, robotHeading;
    private boolean climbAwayFromScoringGrid;

    public ClimbCSAprilTag(double speed, double robotHeading, boolean climbAwayFromScoringGrid){
        drivetrain = Drivetrain.getInstance();
        limelightFront = LimelightFront.getInstance();
        limelightBack = LimelightBack.getInstance();

        addRequirements(drivetrain);
        this.speed = speed;
        this.robotHeading = robotHeading;
        this.climbAwayFromScoringGrid = climbAwayFromScoringGrid;

    }

    @Override
    public void initialize() {
        Translation2d chargeStationVector = new Translation2d(speed, new Rotation2d(Math.toRadians(robotHeading)));
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Below, red side is untested, as is the entire climb towards grid (else case).
        if(climbAwayFromScoringGrid){
            if(DriverStation.getAlliance() == Alliance.Blue){
                return limelightFront.getBotpose().getX() > 3.7;
            }
            else if(DriverStation.getAlliance() == Alliance.Red){
                return limelightFront.getBotpose().getX() < 12.7;
            }
            return true;
        }
        else{
            if(DriverStation.getAlliance() == Alliance.Blue){
                return limelightBack.getBotpose().getX() < 3.9;
            }
            else if(DriverStation.getAlliance() == Alliance.Red){
                return limelightBack.getBotpose().getX() > 12.9;
            }
            return true;
        }
    }

    
}