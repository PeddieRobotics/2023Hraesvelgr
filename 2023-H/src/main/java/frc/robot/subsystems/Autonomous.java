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
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePose;
import frc.robot.commands.ArmCommands.SetTransitoryPoseL3ReturnInAuto;
import frc.robot.commands.ArmCommands.SetLevelThreeConeInvertedPose;
import frc.robot.commands.ArmCommands.SetLevelThreeConePoseInAuto;
import frc.robot.commands.ArmCommands.SetLevelThreeCubeForwardPose;
import frc.robot.commands.ArmCommands.SetLevelTwoCubePose;
import frc.robot.commands.ArmCommands.SetTravelOverBridgePoseInAuto;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.commands.ClawCommands.IntakeFloorCone;
import frc.robot.commands.ClawCommands.IntakeFloorCubeInAuto;
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
    
        eventMap.put("IntakeConePose", new SetExtendedFloorConePose());
        eventMap.put("IntakeCubePose", new SetExtendedFloorCubeInAuto());

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

        //Test Paths
        autoRoutines.put("Loop", autoBuilder.fullAuto(PathPlanner.loadPathGroup("loop", 2.5, 2.5)));

        // Competition paths

        // 1 piece routines without charge station
        autoRoutines.put("1 Piece Bump Column 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceCol1", 1.0, 1.0)));
        autoRoutines.put("1 Piece Top Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceCol9", 1.0, 1.0)));

        // 1 piece routines with charge station
        // autoRoutines.put("1 Piece Bump Column 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBumpCol1", 1.0, 1.0)));
        autoRoutines.put("1 Piece Balance Back Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceBackCol9", 1.5, 1.5)));
        autoRoutines.put("1 Piece Balance Column 3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol3", 1.0, 1.0)));
        autoRoutines.put("1 Piece Balance Column 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol4", 1.0, 1.0)));
        autoRoutines.put("1 Piece Balance Column 6", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol6", 1.0, 1.0)));
        autoRoutines.put("1 Piece Balance Column 7", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol7", 1.0, 1.0)));

        autoRoutines.put("1 Piece Move Balance Column 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceMoveBalanceCol4", 1.0, 3.0)));


        // 2 piece routines without charge station
        // autoRoutines.put("2 Piece Bump Column 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBump", 0.5, 0.5)));
        autoRoutines.put("2 Piece Top Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceTopSweep", 2.0, 2.5)));
        autoRoutines.put("2 Piece Top Column 9 Curve", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceTopCurve", 2.0, 2.5)));
        autoRoutines.put("2 Piece Top Column 9 Pivot", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceTopPivot", 2.0, 2.5)));


        autoRoutines.put("2 Piece Prepare Top Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PiecePrepareTop", 0.5, 0.5)));

        // 2 piece routines with charge station
        autoRoutines.put("2 Piece Balance Top Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBalanceTop", 0.5, 0.5)));

        // 3 piece routines without charge station
        // autoRoutines.put("3 Piece L1 Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("3PieceL1Top", 0.5, 0.5)));

        // autoRoutines.put("3 Piece Balance Top", autobuilder.fullAuto(PathPlanner.loadPathGroup("3PieceBalanceTop", 0.5, 0.5)));
        // autoRoutines.put("3 Piece Top", autobuilder.fullAuto(PathPlanner.loadPathGroup("3PieceTop", 0.5, 0.5)))

        // //test paths
        int i=10;
        while(i-->1){
            autoRoutines.put("TestPath"+i, autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath"+i, 1, 1)));
        }
        autoRoutines.put("TestPath", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath", 0.5, 0.5)));
        // autoRoutines.put("testPath1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath1", 0.5, 0.5)));
        // autoRoutines.put("testPath2", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath2", 0.5, 0.5)));
        // autoRoutines.put("testPath3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath3", 0.5, 0.5)));
        // autoRoutines.put("testPath4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath4", 0.5, 0.5)));

        //Testing auto paths 
        // autoRoutines.put("Testing Auto Path 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("testingautopart1", 1.5, 1.5)));
        // autoRoutines.put("Testing Auto Path 2", autoBuilder.fullAuto(PathPlanner.loadPathGroup("testingautopart2", 1.5, 1.5)));
    }   

    // Used only at the start of autonomous
    private void setFlipped(){ 
        drivetrain.setFlipped();
    }

    public Hashtable<String, Command> getAutoRoutines() {
        return autoRoutines;
    }

}
