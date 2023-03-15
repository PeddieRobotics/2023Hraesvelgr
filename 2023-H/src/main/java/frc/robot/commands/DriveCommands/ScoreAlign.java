package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.DriverOI;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Constants.LimelightConstants;

public class ScoreAlign extends CommandBase {
    private final LimelightFront limelightFront;
    private final LimelightBack limelightBack;
    private final Drivetrain drivetrain;
    private PIDController thetaController, yController;
    private DriverOI oi;
    private Arm arm;
    private Blinkin blinkin;
    private Claw claw;
    private String limelightName;
    private int scoreSetpoint;
    private boolean initialHeadingCorrectionComplete, initialTargetNotFound;

    private double thetaP, yP;

    public ScoreAlign() {
        limelightBack = LimelightBack.getInstance();
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();

        thetaP = 0.09;
        yP = 0.06;

        // SmartDashboard.putNumber("thetaP", thetaP);
        // SmartDashboard.putNumber("yP", yP);

        thetaController = new PIDController(thetaP, 0.0003, 0);
        thetaController.enableContinuousInput(-180, 180);
        yController = new PIDController(yP, 0, 0);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        initialHeadingCorrectionComplete = false;
        initialTargetNotFound = false;

        switch (arm.getState()) {
            case L3_CUBE_INVERTED:
                limelightName = "limelight-back";
                scoreSetpoint = 0;
                break;
            case L3_CONE_INVERTED:
                limelightName = "limelight-back";
                scoreSetpoint = 0;
                break;
            default:
                limelightName = "limelight-front";
                scoreSetpoint = 180;
        }

        blinkin.acquiringTarget();

        oi = DriverOI.getInstance();
        
        // thetaController.setP(SmartDashboard.getNumber("thetaP", 0));
        // yController.setP(SmartDashboard.getNumber("yP", 0));

    }

    @Override
    public void execute() {
        double yMove = 0.0;
        double turn = 0.0;
        double turnFF = 0.2;
        double alignError = claw.getConeAlignmentError();

        double txAvg;
        if (limelightName.equals("limelight-back")) {
            txAvg = limelightBack.getTxAverage();
            alignError = -alignError;
        } else {
            txAvg = limelightFront.getTxAverage();
        }

        if (!initialHeadingCorrectionComplete && Math
                .abs(Math.abs(drivetrain.getHeading()) - scoreSetpoint) > LimelightConstants.kLimelightHeadingBound) {
            turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);

            drivetrain.drive(new Translation2d(oi.getForward() * 1.0, oi.getStrafe() * 1.0),
                    turn + turnFF * Math.signum(turn), true, new Translation2d(0, 0));
        } else if (Math.abs(txAvg) > LimelightConstants.kLimeLightTranslationAngleBound) {
            // If we still don't see a target after the first heading correction stage is complete, stop.
            // Otherwise, proceed indefinitely.
            if (!initialHeadingCorrectionComplete && !LimelightHelper.getTV(limelightName)) {
                blinkin.failure();
                initialTargetNotFound = true;
                return;
            }
            initialHeadingCorrectionComplete = true;

            yMove = yController.calculate(txAvg, alignError);

            drivetrain.drive(new Translation2d(oi.getForward() * 1.0, yMove), 0, true, new Translation2d(0, 0));

        } else {
            drivetrain.drive(new Translation2d(oi.getForward() * 1.0, 0), 0, true, new Translation2d(0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
        if(!claw.hasGamepiece()){
            limelightBack.setPipeline(0);
            limelightFront.setPipeline(7);
        }
        if(!initialTargetNotFound){
            blinkin.returnToRobotState();
        }
    }

    @Override
    public boolean isFinished() {
        return initialTargetNotFound;
    }
}