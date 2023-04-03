package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Hashtable;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.ArmCommands.SetShoulderHomePose;
import frc.robot.commands.ArmCommands.SetTravelOverBridgePoseInAuto;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.ClawCommands.BackwardsConeShot;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.commands.ClawCommands.IntakeFloorCone;
import frc.robot.commands.ClawCommands.IntakeFloorCube;
import frc.robot.commands.DriveCommands.ClimbCSGyro;
import frc.robot.commands.DriveCommands.ClimbCSGyro;
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

    // Auto Builder
    private CustomAutoBuilder autoBuilder;

    // Paths

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        
        // Setup sendable chooser
        autoRoutines = new Hashtable<String, Command>();

        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("stow", new ParallelRaceGroup(new SetStowedPose(), new WaitCommand(3)));
        eventMap.put("eject", new ParallelRaceGroup(new EjectGamepiece(), new WaitCommand(.3)));
        eventMap.put("lock", new LockDrivetrain());
        eventMap.put("homeShoulder", new SetShoulderHomePose());
        eventMap.put("straighten", new StraightenDrivetrain());

        eventMap.put("pidturnto0", new RotateToAngle(0));
        eventMap.put("pidturnto180", new RotateToAngle(180));

        eventMap.put("ConeL2Stowed", new SequentialCommandGroup(new SetLevelTwoConeStowedPose(), new WaitCommand(.3), new BackwardsConeShot(.3)));
        eventMap.put("CubeL2ShotPose", new SetLevelTwoCubeShot());

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
        eventMap.put("IntakeCubePoseLessLower", new SetExtendedFloorCubeInAutoLessLower());
        eventMap.put("IntakeCubePoseLower", new SetExtendedFloorCubeInAutoLower());
        eventMap.put("IntakeCubePoseTeleop", new SetExtendedFloorCubePoseOld());

        eventMap.put("ClimbCSFrontSlow", new SequentialCommandGroup(new ClimbCSGyro(0, 1.0, 0.5), new LockDrivetrain()));
        eventMap.put("ClimbCSBackSlow", new SequentialCommandGroup(new ClimbCSGyro(180, 1.0, 0.5), new LockDrivetrain()));
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
        new PIDConstants(AutoConstants.kPTranslationController, 0, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(AutoConstants.kPThetaController, 0, 0), // PID constants to correct for rotation error (used to create the rotation controller)
            drivetrain::setSwerveModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true,
            drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
        );

        setupAutoRoutines();
    }

    @Override
    public void periodic(){
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

        // 1 piece routines with charge station - dead reckoning / no gyro
        // First two are basic ones from Hatboro. Don't leave community, just balance immediately.
        autoRoutines.put("Hatboro 1 Piece Balance Front Col 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("HATBORO1PieceBalanceFrontCol4", 1.0, 1.0)));

        // 1 piece routines with charge station - GYRO
        // These are just versions of the above that use a gyro-based balancing method rather than dead reckoning.
        autoRoutines.put("Seneca GYRO 1 Piece Balance Back Col 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("SENECAGyro1PieceBalanceBackCol4", 1.25, 3.0)));

        // 2 piece routines without charge station
        autoRoutines.put("Seneca 2 Piece Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("SENECA2PieceCol9", 2, 2)));

        autoRoutines.put("Open 2 Piece Mid Pickup Balance Sweep", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Open2PieceMidPickupBalanceSweep", 2.5, 3)));

        autoRoutines.put("Bump Path 2 piece", autoBuilder.fullAuto(PathPlanner.loadPathGroup("BumpPath", 2.5, 2.5)));
        
        autoRoutines.put("new Bump", new SequentialCommandGroup(
                        autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP1", 2, 2)),
                        autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP2", 1, 1)),
                        autoBuilder.fullAuto(PathPlanner.loadPathGroup("NewBumpP3", 2, 2))));

        // 3 piece routines here


        /*
         * Non-competition paths start here
         */
        // //test paths
        // int i=10;
        // while(i-->1){
        //     autoRoutines.put("TestPath"+i, autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath"+i, 1, 1.5)));
        // }
        autoRoutines.put("Test Path", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath", .5, 1)));
    }   

    // Used only at the start of autonomous
    private void setFlipped(){ 
        drivetrain.setFlipped();
    }

    public Hashtable<String, Command> getAutoRoutines() {
        return autoRoutines;
    }

}
