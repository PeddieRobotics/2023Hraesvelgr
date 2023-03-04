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

public class ClimbCSOdometry extends CommandBase{
    private Drivetrain drivetrain;
    private LimelightFront limelightFront;
    private LimelightBack limelightBack;
    private double speed, robotHeading, initialRobotX, deltaX;
    private boolean climbAwayFromScoringGrid, useLLFront;

    public ClimbCSOdometry(double speed, double robotHeading, boolean climbAwayFromScoringGrid, boolean useLLFront){
        SmartDashboard.putNumber("Climbing Delta X", 0);
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
        deltaX = SmartDashboard.getNumber("Climbing Delta X", 0);
        initialRobotX = drivetrain.getPose().getX();
        Translation2d chargeStationVector = new Translation2d(speed, new Rotation2d(Math.toRadians(robotHeading)));
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());

        if(useLLFront){
            limelightFront.startAveragingX();
        }
        else{
            limelightBack.startAveragingX();
        }
    }

    @Override
    public void execute() {
        Translation2d chargeStationVector = new Translation2d(speed, new Rotation2d(Math.toRadians(robotHeading)));
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getPose().getX() - initialRobotX) > deltaX;
    }

    
}
