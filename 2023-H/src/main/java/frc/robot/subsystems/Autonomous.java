package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/* 
 * TODO: loadPathGroup has been moved to a new function (it appears)
 * TODO: PathConstraints constructor now takes 4 params instead of 2
 * TODO: PathPlanner is removed, there are now PathPlannerPlan, PathPlannerTrajectory, PathPlannerAuto and PathPlannerLogging
 * PathConstraints​(double maxVelocityMps, double maxAccelerationMpsSq, double maxAngularVelocityRps, double maxAngularAccelerationRpsSq)
 */


// import com.pathplanner.lib.PathConstraints; becomes:
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.auto.PIDConstants; becomes:
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// import com.pathplanner.lib.PathPlanner; removed

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.SetExtendedFloorConePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubeInAuto;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubeInAutoLessLower;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubeInAutoLower;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePoseOld;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePoseOldAutonL3;
import frc.robot.commands.ArmCommands.SetLevelOnePose;
import frc.robot.commands.ArmCommands.SetTransitoryPoseL3ReturnInAuto;
import frc.robot.commands.ArmCommands.SetLevelThreeConeInvertedPose;
import frc.robot.commands.ArmCommands.SetLevelThreeConePoseInAuto;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeForwardPose;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeInvertedPoseInAuto;
import frc.robot.commands.ArmCommands.SetLevelTwoConeStowedPose;
import frc.robot.commands.ArmCommands.SetLevelTwoCubePose;
import frc.robot.commands.ArmCommands.SetLevelTwoCubeShot;
import frc.robot.commands.ArmCommands.SetPreScorePose;
import frc.robot.commands.ArmCommands.SetPreScorePoseWristDown;
import frc.robot.commands.ArmCommands.SetTravelOverBridgePoseInAuto;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.ClawCommands.BackwardsConeShot;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.commands.ClawCommands.IntakeFloorCone;
import frc.robot.commands.ClawCommands.IntakeFloorCube;
import frc.robot.commands.DriveCommands.ClimbCSGyro;
import frc.robot.commands.DriveCommands.ClimbCSGyroDelta;
import frc.robot.commands.DriveCommands.ClimbCSGyroWithAnglePid;
import frc.robot.commands.DriveCommands.FollowNoteInAuto;
import frc.robot.commands.DriveCommands.ForcedCalibration;
import frc.robot.commands.DriveCommands.FullOdometryAlign;
import frc.robot.commands.DriveCommands.LockDrivetrain;
import frc.robot.commands.DriveCommands.RotateToAngle;
import frc.robot.commands.DriveCommands.StraightenDrivetrain;
import frc.robot.utils.Constants;
import frc.robot.commands.DriveCommands.AutoDrive;

// import frc.robot.utils.CustomAutoBuilder;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Autonomous extends SubsystemBase{
    // Subsystems
    private static Autonomous autonomous;
    private final Drivetrain drivetrain;
    private final Claw claw;
    private final Arm arm;
    private final LimelightFront limelightFront;
    //private final Claw claw;

    private static SendableChooser<Command> autoChooser;

    private HashMap<String, Command> eventMap;

    // Auto Builder
    // private CustomAutoBuilder autoBuilder;

    // Paths

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        limelightFront = LimelightFront.getInstance();

        // NamedCommands.registerCommand("stow", new ParallelRaceGroup(new SetStowedPose(), new WaitCommand(3)));
        // NamedCommands.registerCommand("eject", new ParallelRaceGroup(new EjectGamepiece(), new WaitCommand(.3)));
        // NamedCommands.registerCommand("lock", new LockDrivetrain());
        NamedCommands.registerCommand("straighten", new StraightenDrivetrain());

        // NamedCommands.registerCommand("pidturnto0", new RotateToAngle(0));
        // NamedCommands.registerCommand("pidturnto180", new RotateToAngle(180));

        // NamedCommands.registerCommand("ConeL2Stowed", new SequentialCommandGroup(new SetLevelTwoConeStowedPose(), new WaitCommand(.3), new BackwardsConeShot(.3)));
        // NamedCommands.registerCommand("CubeL2ShotPose", new SetLevelTwoCubeShot());

        // NamedCommands.registerCommand("L1Pose", new SetLevelOnePose());

        // NamedCommands.registerCommand("IntakeCone", new IntakeFloorCone());
        // NamedCommands.registerCommand("IntakeCube", new IntakeFloorCube());

        // NamedCommands.registerCommand("ConeL3", new SequentialCommandGroup(new SetLevelThreeConePoseInAuto(), new WaitCommand(.3), new InstantCommand(claw::stopClaw),new SetTransitoryPoseL3ReturnInAuto()));

        // NamedCommands.registerCommand("CubeL2Pose", new SetLevelTwoCubePose());
        // NamedCommands.registerCommand("OverBridgePose", new SetTravelOverBridgePoseInAuto());

        // NamedCommands.registerCommand("CubeL3Pose", new SetLevelThreeCubeForwardPose());
        // NamedCommands.registerCommand("ConeL3Pose", new SetLevelThreeConeInvertedPose());

        // NamedCommands.registerCommand("PreScorePose", new SetPreScorePose());
        // NamedCommands.registerCommand("PreScorePoseWristDown", new SetPreScorePoseWristDown());

        // NamedCommands.registerCommand("CubeL3InvertedPose", new SetLevelThreeCubeInvertedPoseInAuto());
        // NamedCommands.registerCommand("CubeL3InvertedPoseReturn", new SetTransitoryPoseL3ReturnInAuto());

        // NamedCommands.registerCommand("IntakeConePose", new SetExtendedFloorConePose());
        // NamedCommands.registerCommand("IntakeCubePose", new SetExtendedFloorCubePoseOld());
        // NamedCommands.registerCommand("IntakeCubePoseFromL3", new SetExtendedFloorCubePoseOldAutonL3());
        // NamedCommands.registerCommand("IntakeCubePoseLessLower", new SetExtendedFloorCubeInAutoLessLower());
        // NamedCommands.registerCommand("IntakeCubePoseLower", new SetExtendedFloorCubeInAutoLower());
        // NamedCommands.registerCommand("IntakeCubePoseTeleop", new SetExtendedFloorCubePoseOld());

        // NamedCommands.registerCommand("ClimbCSFrontSlow", new SequentialCommandGroup(new ClimbCSGyro(0, 1.0, 0.75), new LockDrivetrain()));

        // NamedCommands.registerCommand("ClimbCSBackSlow", new SequentialCommandGroup(new ClimbCSGyroDelta(180, 1.0, 0.75), new LockDrivetrain()));

        // NamedCommands.registerCommand("TranslateRotate", new AutoDrive(new Translation2d(-.3, 0), 0.5 * Constants.DriveConstants.kMaxAngularSpeed));

        NamedCommands.registerCommand("Set Odom", new ForcedCalibration());
        NamedCommands.registerCommand("Follow note", new FollowNoteInAuto());
        NamedCommands.registerCommand("Turn on MegaTag", new InstantCommand(() -> drivetrain.setUseMegaTag(true)));
        NamedCommands.registerCommand("Turn off MegaTag", new InstantCommand(() -> drivetrain.setUseMegaTag(false)));
        NamedCommands.registerCommand("Set Pipeline to 1", new InstantCommand(() -> limelightFront.setPipeline(1)));         // TODO: tune PIDConstants
        NamedCommands.registerCommand("PID back to start", new FullOdometryAlign(2, 5.50, 0, 3));
        NamedCommands.registerCommand("PID to back to midline start", new FullOdometryAlign(5.50, 5.50, 0, 3));

        AutoBuilder.configureHolonomic(
            drivetrain::getPose, // Robot pose supplier
            drivetrain::resetRobotPoseAndGyro, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::getRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drivetrain::driveAuton, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(6, 0.0, 0.0), // Translation PID constants
                new PIDConstants(4.5, 0.0, 0.0), // Rotation PID constants
                Constants.DriveConstants.kMaxFloorSpeed, // Max module speed, in m/s
                Constants.DriveConstants.kBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get()==DriverStation.Alliance.Red;
                }
                return false;
            },
            drivetrain // Reference to drive subsystem to set requirements
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // setupAutoRoutines();

        SmartDashboard.putBoolean("RunAutonWithoutEvents", false);

        // SmartDashboard.putNumber("auto trans p", AutoConstants.kPTranslationController);
        // SmartDashboard.putNumber("auto trans i", AutoConstants.kITranslationController);
        // SmartDashboard.putNumber("auto trans d", AutoConstants.kDTranslationController);

        // SmartDashboard.putNumber("auto theta p", AutoConstants.kPThetaController);
        // SmartDashboard.putNumber("auto theta i", AutoConstants.kIThetaController);
        // SmartDashboard.putNumber("auto theta d", AutoConstants.kDThetaController);

        SmartDashboard.putNumber("Auto Translation P", AutoConstants.kPTranslationController);
        SmartDashboard.putNumber("Auto Rotation P", AutoConstants.kPThetaController);
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Auto Translation P", AutoConstants.kPTranslationController);
        // SmartDashboard.putNumber("Auto Rotation P", AutoConstants.kPThetaController);
    }

    public void resetAutoBuilderAndPaths(){
    }

    public static Autonomous getInstance(){
        if(autonomous == null){
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    @Deprecated
    public void setupAutoRoutines(){
        /*
         * Competition paths start here
         */

        //DEV COMMENT
        ///*

    }   

    // Used only at the start of autonomous
    private void setFlipped(){ 
        drivetrain.setFlipped();
    }

    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
