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

public class SimpleAlign extends CommandBase {
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
    private boolean initialHeadingCorrectionComplete;

    public SimpleAlign() {
        limelightBack = LimelightBack.getInstance();
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();

        thetaController = new PIDController(0.005, 0.0001, 0);
        thetaController.enableContinuousInput(-180, 180);
        yController = new PIDController(0.07, 0.0001, 0);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        initialHeadingCorrectionComplete = false;

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
    }

    @Override
    public void execute() {
        double yMove = 0.0;
        double turn = 0.0;
        double moveFF = 0.05;
        double turnFF = 0.2;
        double alignError = claw.getConeAlignmentError();

        double txAvg;
        if (limelightName.equals("limelight-back")) {
            txAvg = limelightBack.getTxAverage();
            alignError = -alignError;
        } else {
            txAvg = limelightFront.getTxAverage();
        }

        double angularError = Math.abs(Math.abs(drivetrain.getHeading()) - scoreSetpoint);
        SmartDashboard.putNumber("auto-align angular error", angularError);
        if (!initialHeadingCorrectionComplete && angularError > LimelightConstants.kLimelightHeadingBound) {  
                SmartDashboard.putNumber("stage", 1);        
                turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);

                if(Math.abs(txAvg) > 1.0){
                    yMove = yController.calculate(txAvg, alignError);
                }
                else{
                    yMove = 0;
                }

            drivetrain.drive(new Translation2d(oi.getForward() * 0.7, oi.getStrafe() + yMove),
                    turn + turnFF * Math.signum(turn), true, new Translation2d(0, 0));
        } else if (Math.abs(txAvg) > 1.0) {
            SmartDashboard.putNumber("stage", 2);        
            initialHeadingCorrectionComplete = true;

            yMove = yController.calculate(txAvg, alignError);

            drivetrain.drive(new Translation2d(oi.getForward() * 0.7, yMove), 0, true, new Translation2d(0, 0));
        } else {
            SmartDashboard.putNumber("stage", 3);        

            if (Math.abs(Math.abs(drivetrain.getHeading()) - scoreSetpoint) > LimelightConstants.kLimelightHeadingBound) {
                turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);
            }
            else{
                turn = 0;
            }

            drivetrain.drive(new Translation2d(oi.getForward() * 0.7, 0), turn, true, new Translation2d(0, 0));
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
        return false;
    }
}
