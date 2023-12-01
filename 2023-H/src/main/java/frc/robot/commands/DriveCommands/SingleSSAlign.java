package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.DriverOI;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Constants.LimelightConstants;
import frc.robot.subsystems.Blinkin;

public class SingleSSAlign extends Command {
    private final LimelightFront limelightFront;
    private final Drivetrain drivetrain;
    private PIDController thetaController, xController;
    private DriverOI oi;
    private Arm arm;
    private Blinkin blinkin;
    private Claw claw;
    private int scoreSetpoint;
    private boolean initialHeadingCorrectionComplete, initialTargetNotFound;

    public SingleSSAlign() {
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();

        thetaController = new PIDController(0.07, 0.0003, 0);
        thetaController.enableContinuousInput(-180, 180);
        xController = new PIDController(0.07, 0, 0);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        initialHeadingCorrectionComplete = false;
        initialTargetNotFound = false;

        switch (DriverStation.getAlliance().get()) {
            case Red:
                if(drivetrain.getFlipped()){
                    scoreSetpoint = -90;
                }
                else{
                    scoreSetpoint = 90;
                }
                break;
            case Blue:
                if(drivetrain.getFlipped()){
                    scoreSetpoint = 90;
                }
                else{
                    scoreSetpoint = -90;
                }
                break;
            default:
                scoreSetpoint = -90;
        }

        oi = DriverOI.getInstance();

        if (!limelightFront.hasTarget()) {
            blinkin.failure();
        } 
    }

    @Override
    public void execute() {
        double xMove = 0.0;
        double turn = 0.0;
        double turnFF = 0.2;
        double xFF = 0.05;

        Translation2d swerveTranslation = oi.getSwerveTranslation();
        swerveTranslation = swerveTranslation.times(LimelightConstants.kDriveScaleSingleSSAlign);

        double txAvg = limelightFront.getTxAverage();
        if (!initialHeadingCorrectionComplete && Math
                .abs(Math.abs(drivetrain.getHeading()) - scoreSetpoint) > LimelightConstants.kLimelightHeadingBound) {
            turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);

            drivetrain.drive(swerveTranslation,
                    turn + turnFF * Math.signum(turn), true, new Translation2d(0, 0));
        } else if (Math.abs(txAvg) > LimelightConstants.kLimeLightTranslationSingleSSAngleBound) {
            // If we still don't see a target after the first heading correction stage is complete, stop.
            // Otherwise, proceed indefinitely.
            if (!initialHeadingCorrectionComplete){
                if(!LimelightHelper.getTV("limelight-front")) {
                    blinkin.failure();
                    initialTargetNotFound = true;
                    return;
                }
            }
            initialHeadingCorrectionComplete = true;

           xMove = xController.calculate(txAvg, 0);

            drivetrain.drive(new Translation2d(xMove + xFF * Math.signum(xMove), swerveTranslation.getY()), oi.getRotation(), true, new Translation2d(0, 0));

        } else {
            drivetrain.drive(new Translation2d(0, swerveTranslation.getY()), oi.getRotation(), true, new Translation2d(0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
        blinkin.returnToRobotState();

    }

    @Override
    public boolean isFinished() {
        return initialTargetNotFound;
    }
}
