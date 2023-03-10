package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

public class SimpleAlign extends CommandBase{
    private final LimelightFront limelightFront;
    private final LimelightBack limelightBack;
    private final Drivetrain drivetrain;
    private PIDController thetaController, yController;
    private DriverOI oi;
    private Arm arm;
    private Claw claw;
    private String limelightName;
    private int scoreSetpoint;
    private boolean initialHeadingCorrectionComplete;

    public SimpleAlign(){
        limelightBack = LimelightBack.getInstance();
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        
        thetaController = new PIDController(0.05, 0.0001, 0);
        thetaController.enableContinuousInput(-180, 180);
        yController = new PIDController(0.055, 0, 0);

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        initialHeadingCorrectionComplete = false;

        switch (arm.getState()) {
            case L3_CUBE_INVERTED:
                limelightName = "limelightBack";
                scoreSetpoint = 0;
                break;
            case L3_CONE_INVERTED:
                limelightName = "limelightBack";
                scoreSetpoint = 0;
                break;
            default:
                limelightName = "limelightFront";
                scoreSetpoint = 180;
        }

        oi = DriverOI.getInstance();

        if(claw.hasCone()){
            LimelightHelper.setPipelineIndex(limelightName, 6); // Retroreflective tape pipeline
        }
        else{
            LimelightHelper.setPipelineIndex(limelightName, 0); // April tag pipeline
        }
    }

    @Override
    public void execute() {
        double yMove = 0.0;
        double turn = 0.0;
        double turnFF = 0.2;

        double txAvg;
        if(limelightName.equals("limelightBack")){
            txAvg = limelightBack.getTxAverage();
        } else {
            txAvg = limelightFront.getTxAverage();
        }

        if (!initialHeadingCorrectionComplete && Math.abs(Math.abs(drivetrain.getHeading())-scoreSetpoint) > LimelightConstants.kLimelightHeadingBound) {
            turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint); 

            drivetrain.drive(new Translation2d(oi.getForward()*0.3, oi.getStrafe()*0.3), turn+turnFF*Math.signum(turn), true, new Translation2d(0, 0));
        }
        else if(Math.abs(txAvg) > LimelightConstants.kLimeLightTranslationAngleBound){
            initialHeadingCorrectionComplete = true;

            yMove = yController.calculate(txAvg, claw.getConeAlignmentError());

            drivetrain.drive(new Translation2d(oi.getForward()*0.3, yMove), 0, true, new Translation2d(0, 0));

        }
        else {
            if(Math.abs(Math.abs(drivetrain.getHeading())-scoreSetpoint) > LimelightConstants.kLimelightHeadingBound/3){
                turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);
            }
            drivetrain.drive(new Translation2d(oi.getForward()*0.3, 0), turn, true, new Translation2d(0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}