package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Claw.ClawState;
import frc.robot.utils.DriverOI;
import frc.robot.utils.LimelightHelper;
import frc.robot.utils.Constants.LimelightConstants;

public class ScoreAlign extends Command {
    private final LimelightFront limelightFront;
    // private final LimelightBack limelightBack;
    private final Drivetrain drivetrain;
    private PIDController thetaController, yController;
    private DriverOI oi;
    private Arm arm;
    private Blinkin blinkin;
    private Claw claw;
    private String limelightName;
    private int scoreSetpoint;
    private boolean initialHeadingCorrectionComplete, initialTargetNotFound;

    private boolean horizAlignComplete, depthAlignComplete;

    private double successDepth;

    private double convertedGamepieceAlignError;

    public ScoreAlign() {
        // limelightBack = LimelightBack.getInstance();
        limelightFront = LimelightFront.getInstance();
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        blinkin = Blinkin.getInstance();
        claw = Claw.getInstance();

        initialHeadingCorrectionComplete = false;
        initialTargetNotFound = false;
        convertedGamepieceAlignError = 0;

        horizAlignComplete = false;
        depthAlignComplete = false;
        
        successDepth = 0.0;

        thetaController = new PIDController(0.07, 0.0003, 0);
        thetaController.enableContinuousInput(-180, 180);
        yController = new PIDController(0.07, 0, 0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        initialHeadingCorrectionComplete = false;
        initialTargetNotFound = false;
        convertedGamepieceAlignError = 0;

        horizAlignComplete = false;
        depthAlignComplete = false;

        successDepth = 0.0;

        if(arm.getState() == ArmState.L3_CONE_INVERTED || arm.getState() == ArmState.L3_CUBE_INVERTED || arm.getGoalPose() == ArmState.L3_CONE_INVERTED || arm.getGoalPose() == ArmState.L3_CUBE_INVERTED){
            if(drivetrain.getFlipped()){
                scoreSetpoint = 0;
            }
            else{
                scoreSetpoint = 180;
            }
            limelightName = "limelight-back";

        }
        else{
            if(drivetrain.getFlipped()){
                scoreSetpoint = 180;
            }
            else{
                scoreSetpoint = 0;
            }
            limelightName = "limelight-front";
        }

        oi = DriverOI.getInstance();

        thetaController.reset();
        yController.reset();

        ClawState state = claw.getState();
        if(state == ClawState.CUBE) {
            if(arm.getState() == ArmState.L2_CUBE || arm.getGoalPose() == ArmState.L2_CUBE){
                convertedGamepieceAlignError = claw.convertL2CubeTXToAlignmentError(claw.getGamepieceAlignmentError());
            }
            else if(arm.getState() == ArmState.L3_CUBE_FORWARD || arm.getGoalPose() == ArmState.L3_CUBE_FORWARD){
                convertedGamepieceAlignError = claw.convertL3CubeTXToAlignmentError(claw.getGamepieceAlignmentError());
            }
        } else if(state == ClawState.CONE){
            if(arm.getState() == ArmState.L2_CONE || arm.getGoalPose() == ArmState.L2_CONE){
                convertedGamepieceAlignError = claw.convertL2ConeTXToAlignmentError(claw.getGamepieceAlignmentError());
            }
            else if(arm.getState() == ArmState.L3_CONE_INVERTED || arm.getGoalPose() == ArmState.L3_CONE_INVERTED){
                convertedGamepieceAlignError = claw.convertL3ConeTXToAlignmentError(claw.getGamepieceAlignmentError());
            }
            else if(arm.getState() == ArmState.L3_CONE_FORWARD|| arm.getGoalPose() == ArmState.L3_CONE_FORWARD){
                convertedGamepieceAlignError = claw.convertL2ConeTXToAlignmentError(claw.getGamepieceAlignmentError());
            }
        }

    }

    @Override
    public void execute() {
        double yMove = 0.0;
        double turn = 0.0;
        double turnFF = 0.2;
        double yFF = 0.05;

        Translation2d swerveTranslation = oi.getSwerveTranslation();
        swerveTranslation = swerveTranslation.times(LimelightConstants.kDriveScaleScoreAlign);

        // double txAvg;
        // if (limelightName.equals("limelight-back")) {
        //     txAvg = limelightBack.getTxAverage();
        // } else {
        //     txAvg = limelightFront.getTxAverage();
        // }

        // SmartDashboard.putNumber("converted gamepiece align error", convertedGamepieceAlignError);

        // if (!initialHeadingCorrectionComplete && Math.abs(Math.abs(drivetrain.getHeading()) - scoreSetpoint) > LimelightConstants.kLimelightHeadingBound) {
        //     turn = thetaController.calculate(drivetrain.getHeading(), scoreSetpoint);

        //     drivetrain.drive(swerveTranslation, turn + turnFF * Math.signum(turn), true, new Translation2d(0, 0));
        // }
        //else
        // if (Math.abs(txAvg-convertedGamepieceAlignError) > LimelightConstants.kLimeLightTranslationScoringAngleBound) {
            // If we still don't see a target after the first heading correction stage is complete, stop.
            // Otherwise, proceed indefinitely.
            // if (!initialHeadingCorrectionComplete){
            //     if(!LimelightHelper.getTV(limelightName)) {
            //         blinkin.failure();
            //         initialTargetNotFound = true;
            //         return;
            //     }
            // }
            // initialHeadingCorrectionComplete = true;

        //     yMove = yController.calculate(txAvg, convertedGamepieceAlignError);

        //     drivetrain.drive(new Translation2d(swerveTranslation.getX(), yMove + yFF * Math.signum(yMove)), oi.getRotation(), true, new Translation2d(0, 0));

        // } else {
        //     horizAlignComplete = true;
        //     drivetrain.drive(new Translation2d(swerveTranslation.getX(), 0), oi.getRotation(), true, new Translation2d(0, 0));
        // }

        // Check for how close we are to the goal according to our pose state
        // ClawState state = claw.getState();
        // double tyAvg = 0;

        // if(state == ClawState.CUBE) {
        //     if(arm.getState() == ArmState.L2_CUBE || arm.getGoalPose() == ArmState.L2_CUBE){
        //         tyAvg = limelightFront.getTyAverage();
        //         if(tyAvg < -12){
        //             depthAlignComplete = true;
        //             successDepth = -13;
        //         }
        //     }
        //     else if(arm.getState() == ArmState.L3_CUBE_FORWARD || arm.getGoalPose() == ArmState.L3_CUBE_FORWARD){
        //         tyAvg = limelightFront.getTyAverage();
        //         if(tyAvg < -13){
        //             depthAlignComplete = true;
        //             successDepth = -14;
        //         }
        //     }
        // } else if(state == ClawState.CONE){
        //     if(arm.getState() == ArmState.L2_CONE || arm.getGoalPose() == ArmState.L2_CONE || arm.getState() == ArmState.L3_CONE_FORWARD || arm.getGoalPose() == ArmState.L3_CONE_FORWARD){
        //         tyAvg = limelightFront.getTyAverage();
        //         if(tyAvg < -5.5){
        //             depthAlignComplete = true;
        //             successDepth = -6.5;
        //         }
        //     }
        //     else if(arm.getState() == ArmState.L3_CONE_INVERTED || arm.getGoalPose() == ArmState.L3_CONE_INVERTED){
        //         tyAvg = limelightBack.getTyAverage();
        //         if(tyAvg > 17.5){
        //             depthAlignComplete = true;
        //             successDepth = 18.0;
        //         }
        //     }
        // }

        // double alignmentDist = Math.sqrt(Math.pow(Math.abs(txAvg-convertedGamepieceAlignError), 2) + Math.pow(Math.abs(tyAvg-successDepth), 2));
        // claw.setCurrentAlignmentDistance(alignmentDist);

        // If both flags are up and distance is sufficiently close, change to solid green 
        if(horizAlignComplete){
            blinkin.autoAlignSuccess();
        }
        // Update LED's according to how many stages of the alignment have been completed
        // else if(horizAlignComplete || depthAlignComplete){
        //     blinkin.autoAlignClose();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();

        if(!claw.hasGamepiece()){
            claw.returnLimelightToDefaultState();
        }

        // blinkin.returnToRobotState();

    }

    @Override
    public boolean isFinished() {
        return initialTargetNotFound;
    }
}