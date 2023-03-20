package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Hashtable;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmCommands.SetExtendedFloorConePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubeInAuto;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubeInAutoLower;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePose;
import frc.robot.commands.ArmCommands.SetTransitoryPoseL3ReturnInAuto;
import frc.robot.commands.ArmCommands.SetLevelThreeConeInvertedPose;
import frc.robot.commands.ArmCommands.SetLevelThreeConePoseInAuto;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeForwardPose;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeInvertedPoseInAuto;
import frc.robot.commands.ArmCommands.SetLevelTwoCubePose;
import frc.robot.commands.ArmCommands.SetPreScorePose;
import frc.robot.commands.ArmCommands.SetPreScorePoseWristDown;
import frc.robot.commands.ArmCommands.SetTravelOverBridgePoseInAuto;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.commands.ClawCommands.IntakeFloorCone;
import frc.robot.commands.ClawCommands.IntakeFloorCube;
import frc.robot.commands.ClawCommands.IntakeFloorCubeInAuto;
import frc.robot.commands.DriveCommands.ClimbCSGyro;
import frc.robot.commands.DriveCommands.LockDrivetrain;
import frc.robot.commands.DriveCommands.StraightenDrivetrain;
import frc.robot.utils.CustomAutoBuilder;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Autonomous extends SubsystemBase{
    // Subsystems
    private static Autonomous autonomous;
    private final Drivetrain drivetrain;
    private final Arm arm;
    //private final Claw claw;

    private Hashtable<String, Command> autoRoutines;

    // Auto Builder
    private CustomAutoBuilder autoBuilder;

    // Paths

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        
        // Setup sendable chooser
        autoRoutines = new Hashtable<String, Command>();

        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("stow", new SetStowedPose());
        eventMap.put("eject", new EjectGamepiece());
        eventMap.put("lock", new LockDrivetrain());
        eventMap.put("straighten", new StraightenDrivetrain());

        eventMap.put("IntakeCone", new IntakeFloorCone());
        eventMap.put("IntakeCube", new IntakeFloorCubeInAuto());

        eventMap.put("ConeL3", new SequentialCommandGroup(new SetLevelThreeConePoseInAuto(), new SetTransitoryPoseL3ReturnInAuto()));

        eventMap.put("CubeL2Pose", new SetLevelTwoCubePose());
        eventMap.put("OverBridgePose", new SetTravelOverBridgePoseInAuto());

        eventMap.put("CubeL3Pose", new SetLevelThreeCubeForwardPose());
        eventMap.put("ConeL3Pose", new SetLevelThreeConeInvertedPose());

        eventMap.put("PreScorePose", new SetPreScorePose());
        eventMap.put("PreScorePoseWristDown", new SetPreScorePoseWristDown());

        eventMap.put("CubeL3InvertedPose", new SetLevelThreeCubeInvertedPoseInAuto());
        eventMap.put("CubeL3InvertedPoseReturn", new SetTransitoryPoseL3ReturnInAuto());

        eventMap.put("IntakeConePose", new SetExtendedFloorConePose());
        eventMap.put("IntakeCubePose", new SetExtendedFloorCubeInAuto());
        eventMap.put("IntakeCubePoseLower", new SetExtendedFloorCubeInAutoLower());
        eventMap.put("IntakeCubePoseTeleop", new SetExtendedFloorCubePose());

        eventMap.put("ClimbCSFrontSlow", new ClimbCSGyro(0, 1.0, 0.5));
        eventMap.put("ClimbCSBackSlow", new ClimbCSGyro(180, 1.0, 0.5));
        
        eventMap.put("ClimbCSFrontMedium", new ClimbCSGyro(0, 1.5, 0.5));
        eventMap.put("ClimbCSBackMedium", new ClimbCSGyro(180, 1.5, 0.5));

        eventMap.put("ClimbCSFrontFast", new ClimbCSGyro(0, 2.0, 0.5));
        eventMap.put("ClimbCSBackFast", new ClimbCSGyro(180, 2.0, 0.5));


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

        // 1 piece routines without charge station (leave community and collect a piece as minimum)
        autoRoutines.put("1 Piece Col 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceCol1", 1.5, 1.5)));
        autoRoutines.put("1 Piece Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceCol9", 2, 2.5)));

        // 1 piece routines with charge station - dead reckoning / no gyro
        // First two are basic ones from Hatboro. Don't leave community, just balance immediately.
        autoRoutines.put("1 Piece Balance Front Col 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceFrontCol4", 1.0, 1.0)));
        autoRoutines.put("1 Piece Balance Front Col 6", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceFrontCol6", 1.0, 1.0)));
       
        // These three are upgrades which leave community, collect a piece, and loop from the back onto the charge station.
        autoRoutines.put("1 Piece Balance Back Col 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceBackCol4", 1.0, 3.0)));
        autoRoutines.put("1 Piece Balance Back Col 6", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceBackCol6", 1.0, 3.0)));
        autoRoutines.put("1 Piece Balance Back Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceBackCol9", 2.0, 2.5)));

        // 1 piece routines with charge station - GYRO
        // These are just versions of the above that use a gyro-based balancing method rather than dead reckoning.
        autoRoutines.put("GYRO 1 Piece Balance Front Col 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Gyro1PieceBalanceFrontCol4", 1.0, 1.0)));
        autoRoutines.put("GYRO 1 Piece Balance Front Col 6", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Gyro1PieceBalanceFrontCol6", 1.0, 1.0)));
       
        autoRoutines.put("GYRO 1 Piece Balance Back Col 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Gyro1PieceBalanceBackCol4", 1.5, 3.0)));
        autoRoutines.put("GYRO 1 Piece Balance Back Col 6", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Gyro1PieceBalanceBackCol6", 1.5, 3.0)));
        autoRoutines.put("GYRO 1 Piece Balance Back Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Gyro1PieceBalanceBackCol9", 1.5, 2.0)));

        // 2 piece routines without charge station
        autoRoutines.put("2 Piece Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceCol9", 2.0, 3)));
        autoRoutines.put("2 Piece Col 9 Pivot", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceCol9Pivot", 2.5, 3)));
        

        // 2 piece routines with charge station - dead reckoning / no gyro
        // autoRoutines.put("2 Piece Balance Front Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBalanceFrontCol9", 2.5, 3)));
        // autoRoutines.put("2 Piece Balance Back Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBalanceBackCol9", 2.5, 4)));

        // 2 piece routines with charge station - GYRO
        // autoRoutines.put("GYRO 2 Piece Balance Front Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Gyro2PieceBalanceFrontCol9", 2.5, 3)));
        // autoRoutines.put("GYRO 2 Piece Balance Back Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Gyro2PieceBalanceBackCol9", 2.5, 4)));

        // 2.5 piece routines without charge station
        autoRoutines.put("2 Piece Collect Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceCollectCol9", 1.5, 1.5)));

        // 2.5 piece routines with charge station - dead reckoning / no gyro
        // autoRoutines.put("2 Piece Collect Balance Back Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceCollectBalanceBackCol9", 2.5, 4.5)));

        // 2.5 piece routines with charge station - GYRO
        // autoRoutines.put("GYRO 2 Piece Collect Balance Back Col 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Gyro2PieceCollectBalanceBackCol9", 2.5, 4.5)));


        /*
         * Non-competition paths start here
         */
        // //test paths
        // int i=10;
        // while(i-->1){
        //     autoRoutines.put("TestPath"+i, autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath"+i, 2.5, 2.5)));
        // }
        // autoRoutines.put("TestPath", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath", 0.5, 0.5)));
        // autoRoutines.put("testPath1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath1", 0.5, 0.5)));
        // autoRoutines.put("testPath2", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath2", 0.5, 0.5)));
        // autoRoutines.put("testPath3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath3", 0.5, 0.5)));
        // autoRoutines.put("testPath4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath4", 0.5, 0.5)));

    }   

    // Used only at the start of autonomous
    private void setFlipped(){ 
        drivetrain.setFlipped();
    }

    public Hashtable<String, Command> getAutoRoutines() {
        return autoRoutines;
    }

}
