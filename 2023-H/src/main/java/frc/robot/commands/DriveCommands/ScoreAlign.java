package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
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

    public ScoreAlign() {
        limelightBack = LimelightBack.getInstance();
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();

        thetaController = new PIDController(0.05, 0.0002, 0);
        thetaController.enableContinuousInput(-180, 180);
        thetaController.setIntegratorRange(-0.1, 0.1);
        yController = new PIDController(0.075, 0, 0);

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

        // Only proceed if we see a target, otherwise fail on purpose and give up.
        if (!LimelightHelper.getTV(limelightName)) {
            blinkin.failure();
            initialTargetNotFound = true;
        } else{
            blinkin.acquiringTarget();
        }

        oi = DriverOI.getInstance();
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
        
        if (!initialHeadingCorrectionComplete &&
            Math.abs(Math.abs(drivetrain.getHeading()) - scoreSetpoint) > LimelightConstants.kLimelightHeadingBound) {
            
                turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);
                yMove = yController.calculate(txAvg, alignError);

                drivetrain.drive(new Translation2d(oi.getForward() * 0.4, yMove * Math.signum(yMove) * Math.signum(-turn)),
                    turn + turnFF * Math.signum(turn), true, new Translation2d(0, 0));

        } else if (Math.abs(txAvg) > 0.2) {
            initialHeadingCorrectionComplete = true;

            yMove = yController.calculate(txAvg, alignError);

            turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);

            drivetrain.drive(new Translation2d(oi.getForward() * 0.4, yMove), 0, true, new Translation2d(0, 0));

        } else {
            if (Math.abs(Math.abs(drivetrain.getHeading()) - scoreSetpoint) > LimelightConstants.kLimelightHeadingBound/3) {
                turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);
            }

            drivetrain.drive(new Translation2d(oi.getForward() * 0.4, 0), turn, true, new Translation2d(0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
        if(!claw.hasGamepiece()){
            limelightBack.setPipeline(0);
            limelightFront.setPipeline(7);
        }
        blinkin.returnToRobotState();
    }

    @Override
    public boolean isFinished() {
        return initialTargetNotFound;
    }
}
