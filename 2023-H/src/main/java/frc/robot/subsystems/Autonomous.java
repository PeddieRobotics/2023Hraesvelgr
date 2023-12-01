package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Hashtable;

/* 
 * TODO: loadPathGroup has been moved to a new function (it appears)
 * TODO: PathConstraints constructor now takes 4 params instead of 2
 * TODO: PathPlanner is removed, there are now PathPlannerPlan, PathPlannerTrajectory, PathPlannerAuto and PathPlannerLogging
 * PathConstraints​(double maxVelocityMps, double maxAccelerationMpsSq, double maxAngularVelocityRps, double maxAngularAccelerationRpsSq)
 */


// import com.pathplanner.lib.PathConstraints; becomes:
import com.pathplanner.lib.path.PathConstraints;

// import com.pathplanner.lib.auto.PIDConstants; becomes:
import com.pathplanner.lib.util.PIDConstants;

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
import frc.robot.commands.DriveCommands.LockDrivetrain;
import frc.robot.commands.DriveCommands.RotateToAngle;
import frc.robot.commands.DriveCommands.StraightenDrivetrain;
import frc.robot.utils.CustomAutoBuilder;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Autonomous extends SubsystemBase{
    // Subsystems
    private static Autonomous autonomous;
    private final Drivetrain drivetrain;
    private final Claw claw;
    private final Arm arm;
    //private final Claw claw;

    private Hashtable<String, Command> autoRoutines;

    private HashMap<String, Command> eventMap;

    // Auto Builder
    private CustomAutoBuilder autoBuilder;

    // Paths

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        
        // Setup sendable chooser
        autoRoutines = new Hashtable<String, Command>();

        eventMap = new HashMap<>();

        eventMap.put("stow", new ParallelRaceGroup(new SetStowedPose(), new WaitCommand(3)));
        eventMap.put("eject", new ParallelRaceGroup(new EjectGamepiece(), new WaitCommand(.3)));
        eventMap.put("lock", new LockDrivetrain());
        eventMap.put("straighten", new StraightenDrivetrain());

        eventMap.put("pidturnto0", new RotateToAngle(0));
        eventMap.put("pidturnto180", new RotateToAngle(180));

        eventMap.put("ConeL2Stowed", new SequentialCommandGroup(new SetLevelTwoConeStowedPose(), new WaitCommand(.3), new BackwardsConeShot(.3)));
        eventMap.put("CubeL2ShotPose", new SetLevelTwoCubeShot());

        eventMap.put("L1Pose", new SetLevelOnePose());

        eventMap.put("IntakeCone", new IntakeFloorCone());
        eventMap.put("IntakeCube", new IntakeFloorCube());

        eventMap.put("ConeL3", new SequentialCommandGroup(new SetLevelThreeConePoseInAuto(), new WaitCommand(.3), new InstantCommand(claw::stopClaw),new SetTransitoryPoseL3ReturnInAuto()));

        eventMap.put("CubeL2Pose", new SetLevelTwoCubePose());
        eventMap.put("OverBridgePose", new SetTravelOverBridgePoseInAuto());

        eventMap.put("CubeL3Pose", new SetLevelThreeCubeForwardPose());
        eventMap.put("ConeL3Pose", new SetLevelThreeConeInvertedPose());

        eventMap.put("PreScorePose", new SetPreScorePose());
        eventMap.put("PreScorePoseWristDown", new SetPreScorePoseWristDown());

        eventMap.put("CubeL3InvertedPose", new SetLevelThreeCubeInvertedPoseInAuto());
        eventMap.put("CubeL3InvertedPoseReturn", new SetTransitoryPoseL3ReturnInAuto());

        eventMap.put("IntakeConePose", new SetExtendedFloorConePose());
        eventMap.put("IntakeCubePose", new SetExtendedFloorCubePoseOld());
        eventMap.put("IntakeCubePoseFromL3", new SetExtendedFloorCubePoseOldAutonL3());
        eventMap.put("IntakeCubePoseLessLower", new SetExtendedFloorCubeInAutoLessLower());
        eventMap.put("IntakeCubePoseLower", new SetExtendedFloorCubeInAutoLower());
        eventMap.put("IntakeCubePoseTeleop", new SetExtendedFloorCubePoseOld());

        eventMap.put("ClimbCSFrontSlow", new SequentialCommandGroup(new ClimbCSGyro(0, 1.0, 0.75), new LockDrivetrain()));
        eventMap.put("ClimbCSBackSlow", new SequentialCommandGroup(new ClimbCSGyroDelta(180, 1.0, 0.75), new LockDrivetrain()));
        // eventMap.put("ClimbCSBackSlow", new SequentialCommandGroup(new ClimbCSGyroNew(180, 1.0, 0.5), new WaitCommand(0.5), new ConditionalCommand(new RotateToAngle(180), new InstantCommand(), drivetrain::isBalanced), new LockDrivetrain()));
        
        // eventMap.put("ClimbCSFrontMedium", new ClimbCSGyro(0, 1.5, 0.5));//speed should be 1.0
       
        // eventMap.put("ClimbCSBackMedium", new ClimbCSGyro(180, 1.5, 0.5));

        // eventMap.put("ClimbCSFrontFast", new ClimbCSGyro(0, 2.0, 1.0));
        // eventMap.put("ClimbCSBackFast", new ClimbCSGyro(180, 2.0, 1.0));

        autoBuilder = new CustomAutoBuilder(
        drivetrain ::getPose, // Pose2d supplier
        drivetrain ::resetRobotPoseAndGyro, // Pose2d consumer, used to reset odometry at the beginning of auto
        () -> this.setFlipped(),
        DriveConstants.kinematics, // SwerveDriveKinematics
        new PIDConstants(AutoConstants.kPTranslationController, AutoConstants.kITranslationController, AutoConstants.kDTranslationController), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController), // PID constants to correct for rotation error (used to create the rotation controller)
            drivetrain::setSwerveModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true,
            drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
        );

        setupAutoRoutines();

        SmartDashboard.putBoolean("RunAutonWithoutEvents", false);

        SmartDashboard.putNumber("auto trans p", AutoConstants.kPTranslationController);
        SmartDashboard.putNumber("auto trans i", AutoConstants.kITranslationController);
        SmartDashboard.putNumber("auto trans d", AutoConstants.kDTranslationController);

        SmartDashboard.putNumber("auto theta p", AutoConstants.kPThetaController);
        SmartDashboard.putNumber("auto theta i", AutoConstants.kIThetaController);
        SmartDashboard.putNumber("auto theta d", AutoConstants.kDThetaController);
    }

    @Override
    public void periodic(){
    }

    public void resetAutoBuilderAndPaths(){
        autoBuilder = new CustomAutoBuilder(
            drivetrain ::getPose, // Pose2d supplier
            drivetrain ::resetRobotPoseAndGyro, // Pose2d consumer, used to reset odometry at the beginning of auto
            () -> this.setFlipped(),
            DriveConstants.kinematics, // SwerveDriveKinematics
            new PIDConstants(SmartDashboard.getNumber("auto trans p", 0.0), SmartDashboard.getNumber("auto trans i", 0.0), SmartDashboard.getNumber("auto trans d", 0.0)), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(SmartDashboard.getNumber("auto theta p", 0.0), SmartDashboard.getNumber("auto theta i", 0.0), SmartDashboard.getNumber("auto theta d", 0.0)), // PID constants to correct for rotation error (used to create the rotation controller)
                drivetrain::setSwerveModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true,
                drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
            );
    
            setupAutoRoutines();
    }

    public static Autonomous getInstance(){
        if(autonomous == null){
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    public void setupAutoRoutines(){
        /*
         * Competition paths start here
         */

        //DEV COMMENT
        ///*

        // 1 piece routines with charge station - dead reckoning / no gyro
        autoRoutines.put("1 Piece L3 Center Balance Front", autoBuilder.fullAuto(PathPlanner.loadPathGroup("HATBORO1PieceBalanceFrontCol4", 1.0, 1.0)));

        // 1 piece routines with charge station - GYRO
        autoRoutines.put("GYRO 1.5 Piece L3 Center Balance Back", autoBuilder.fullAuto(PathPlanner.loadPathGroup("SENECAGyro1PieceBalanceBackCol4", 1.25, 3.0)));

        // 2 piece routines without charge station
        // autoRoutines.put("Seneca Open 2 Piece L3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("SENECA2PieceCol9", 2, 2)));
        // autoRoutines.put("Open 2 Piece L3 shift", autoBuilder.fullAuto(PathPlanner.loadPathGroup("SENECA2PieceCol9Shift", 2, 2)));
        // autoRoutines.put("Open 2.5 Piece L3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Modified2PieceCol9", 2.5, 3)));

        //autoRoutines.put("Open 2.5 Piece L2", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Open2PieceMidPickupBalanceSweep", 2.5, 3)));
        //autoRoutines.put("Open 2.5 Piece L2 No Arm", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Open2PieceMidPickupBalanceSweepNoArm", 2.5, 3)));

        // autoRoutines.put("Bump 2 piece L3+L1", new SequentialCommandGroup(
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP1", 2.5, 2.5)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP2Low", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP3Low", 2.5, 2.5))));
        //*/ //DEV COMMENT

        // autoRoutines.put("Bump 2 piece L3 Straight", new SequentialCommandGroup(
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP1Straight", 2.5, 2.5)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP2Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP3Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP4Straight", 2, 1))));

        // autoRoutines.put("180 degree pivot test 1 m/s", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMPPivotTest1",
        // new PathConstraints(1, 1),
        // new PathConstraints(1, 1),
        // new PathConstraints(1, 1)
        // ))));
            
        // autoRoutines.put("180 degree pivot test 1.5 m/s", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMPPivotTest2",
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(1.5, 1.5)
        // ))));
            
        // autoRoutines.put("180 degree pivot test 0.5 m/s", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMPPivotTest3",
        // new PathConstraints(0.5, 1),
        // new PathConstraints(0.5, 1),
        // new PathConstraints(0.5, 1)
        // ))));
            
        // autoRoutines.put("WCMP 3 piece free (cheat)", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMP3FreeCheat",
        //     new PathConstraints(3.5, 3.5),
        //     new PathConstraints(1, 2.5),
        //     new PathConstraints(1, 1),
        //     new PathConstraints(3.5, 2.5),
        //     new PathConstraints(1, 1),
        //     new PathConstraints(2.5, 3),
        //     new PathConstraints(1, 1),
        //     new PathConstraints(3.5, 2.5)
        //     ))));

        // autoRoutines.put("WCMP 2.5 piece free", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMP2FreeCollect1",
        // new PathConstraints(3, 2.5),
        // new PathConstraints(1, 2.5),
        // new PathConstraints(1, 1),
        // new PathConstraints(3,2.5),
        // new PathConstraints(1, 1),
        // new PathConstraints(3, 2.5)
        // ))));

        autoRoutines.put("WCMP 2 piece free (L3)", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMP2Free",
        new PathConstraints(3, 2.5),
        new PathConstraints(1, 2.5),
        new PathConstraints(1, 1),
        new PathConstraints(3,2.5)
        ))));

        autoRoutines.put("WCMP 2 piece bump", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMPFridayTest",
        new PathConstraints(1.5, 2.75)
        ))));

        // autoRoutines.put("WCMP 2 piece bump", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMPFridayTest",
        // new PathConstraints(1.5, 2.75),
        // new PathConstraints(1.5, 2.75),
        // new PathConstraints(1.5, 2.75),
        // new PathConstraints(1, 1),
        // new PathConstraints(1.5, 2.75),
        // new PathConstraints(1.5, 2.75)
        // ))));
            
        // autoRoutines.put("WCMP 3 piece free (v2 cheat)", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMP3FreeCheat2",
        // new PathConstraints(3.5, 3.5),
        // new PathConstraints(1.5, 2),
        // new PathConstraints(3.5, 3.5),
        // new PathConstraints(1.5,2),
        // new PathConstraints(2.5, 3),
        // new PathConstraints(1.5, 2),
        // new PathConstraints(3.5, 3.5)
        // ))));
            
        // autoRoutines.put("WCMP 3 piece free (backward cone)", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMP3FreeBackward",
        // new PathConstraints(3.5, 3.5),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(3.5, 3.5),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(3.5, 3.5),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(2.5, 3),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(3.5, 3.5)
        // ))));

        // autoRoutines.put("WCMP 2.5 piece free (backward cone)", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMP2FreeBackwardCollect1",
        // new PathConstraints(3.5, 3.5),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(3.5, 3.5),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(3.5, 3.5),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(2.5, 3),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(3.5, 3.5)
        // ))));
            
        // autoRoutines.put("WCMP 2 piece bump", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMP2BumpSlanted",
        // new PathConstraints(1.5, 2.5),
        // new PathConstraints(1.5, 2.5),
        // new PathConstraints(1.5, 2.5),
        // new PathConstraints(1, 1),
        // new PathConstraints(1.5, 2.5),
        // new PathConstraints(1.5, 2.5)
        // ))));

        // autoRoutines.put("WCMP 2 piece center balance (cheat)", autoBuilder.fullAuto((PathPlanner.loadPathGroup("WCMP2CenterCheat",
        // new PathConstraints(2, 3),
        // new PathConstraints(0.75, 0.75),
        // new PathConstraints(2, 2),
        // new PathConstraints(1, 1),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(1.5, 1.5),
        // new PathConstraints(3, 3)
        // ))));

        // autoRoutines.put("Bump 2 piece L3 Straight Uncomped", new SequentialCommandGroup(
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP1Straight", 2.5, 2.5)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP2StraightUncomped", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP3Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP4Straight", 2, 1))));

        // autoRoutines.put("Bump 2 piece L3 Straight Short", new SequentialCommandGroup(
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP1Straight", 2.5, 2.5)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP2Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP3Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP4StraightShort", 2, 1))));

        // autoRoutines.put("Bump 2 piece L3 Straight Uncomped Short", new SequentialCommandGroup(
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP1Straight", 2.5, 2.5)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP2StraightUncomped", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP3Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP4StraightShort", 2, 1))));

        // autoRoutines.put("Bump 2 piece L3 Straight Short Twist", new SequentialCommandGroup(
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP1Straight", 2.5, 2.5)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP2Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP3Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP4StraightShortTwist", 2, 1))));

        // autoRoutines.put("Bump 2 piece L3 Straight Uncomped Short Twist", new SequentialCommandGroup(
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP1Straight", 2.5, 2.5)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP2StraightUncomped", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP3Straight", 1, 1)),
        //                 autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP4StraightShortTwist", 2, 1))));

        // 3 piece routines here
        // autoRoutines.put("Open 3 Piece L1/L3/L2 Cheat", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Open3PieceCheat", 2, 2.5)));


        /*
         * Non-competition paths start heredriver
         */
        // //test paths
        //  int i=10;
        //  while(i-->1){
        //     autoRoutines.put("TestPath"+i, autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath"+i, 1, 1.5)));
        //  }
        // autoRoutines.put("Test Path", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath", .5, 1)));
    }   

    // Used only at the start of autonomous
    private void setFlipped(){ 
        drivetrain.setFlipped();
    }

    public Hashtable<String, Command> getAutoRoutines() {
        return autoRoutines;
    }

}
