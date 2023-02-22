package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.SetCompactFloorConePose;
import frc.robot.commands.ArmCommands.SetCompactFloorCubePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePose;
import frc.robot.commands.ArmCommands.SetLevelThreeConePose;
import frc.robot.commands.ArmCommands.SetLevelThreeCubePose;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.AutoCommands.AutonAlign;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.commands.ClawCommands.IntakeCone;
import frc.robot.commands.ClawCommands.IntakeCube;
import frc.robot.commands.LimelightCommands.SetPipe;
import frc.robot.commands.LimelightCommands.SetPipeType;
import frc.robot.utils.CustomAutoBuilder;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Autonomous extends SubsystemBase{
    // Subsystems
    private static Autonomous autonomous;
    private final Drivetrain drivetrain;

    // Sendable Chooser
    private SendableChooser<Command> autoRoutineSelector;
    private Hashtable<String, Command> autoRoutines;

    // Auto Builder
    private CustomAutoBuilder autoBuilder;

    // Paths
    private PathPlannerTrajectory OnePieceBalanceCol3, OnePieceBalanceCol7, OnePieceSkedaddleMid, OnePiecePrepareCol9;
    private PathPlannerTrajectory TwoPieceFree, TwoPieceBump, TwoPieceFreeBalance, TwoPieceFreePrepareCol1, TwoPieceBumpPrepareCol9;
    private PathPlannerTrajectory ThreePieceFree;

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        
        // Setup sendable chooser
        autoRoutines = new Hashtable<String, Command>();
        autoRoutineSelector = new SendableChooser<Command>();

        HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("dummyDashboardCommand", new DummyDashboardCommand());
        // eventMap.put("DeployIntake", new DummyDeployIntakeCommand());
        // eventMap.put("AimAndShoot", new DummyAimAndShootCommand()); //eventMap.put("AimAndShoot", new DummyAimAndShootCommand());
        for(int i=1;i<=9;i++){
            eventMap.put("AlignCol"+i, new AutonAlign(i));
            eventMap.put("pipe"+i, new SetPipe(i));
            eventMap.put("pipeType"+i, new SetPipeType(i));
        }
        eventMap.put("pipe0", new SetPipe(0));

        eventMap.put("ConeL3", new SequentialCommandGroup(new SetLevelThreeConePose(), new EjectGamepiece(),new SetStowedPose()));
        eventMap.put("CubeL3", new SequentialCommandGroup(new SetLevelThreeCubePose(), new EjectGamepiece(),new SetStowedPose()));
        eventMap.put("IntakeCube", new SequentialCommandGroup(new SetExtendedFloorCubePose(), new ParallelRaceGroup( new IntakeCube(), new WaitCommand(3)),new SetStowedPose()));
        eventMap.put("IntakeCone", new SequentialCommandGroup(new SetCompactFloorConePose(), new IntakeCone(),new SetStowedPose()));

        eventMap.put("IntakeConePose", new SetCompactFloorConePose());
        eventMap.put("IntakeCubePose", new SetCompactFloorCubePose());
        eventMap.put("StartIntakingCube", new SequentialCommandGroup( new ParallelRaceGroup( new IntakeCube(), new WaitCommand(4)),new SetStowedPose()));


        // autoBuilder = new SwerveAutoBuilder(
        //     drivetrain ::getPose, drivetrain ::resetRobotPoseAndGyro, 
        //     DriveConstants.kinematics, 
        //     new PIDConstants(AutoConstants.kPTranslationController, 0, 0), 
        //     new PIDConstants(AutoConstants.kPThetaController, 0, 0), 
        //     drivetrain::setSwerveModuleStates, 
        //     eventMap, true, 
        //     drivetrain
        // );

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

        defineAutoPaths();
        setupAutoRoutines();
        setupAutoSelector();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("path intial pose x", PathPlanner.loadPathGroup("SwerveTest", 1.5, 1.5).get(0).getInitialHolonomicPose().getX());
        SmartDashboard.putNumber("path intial pose y", PathPlanner.loadPathGroup("SwerveTest", 1.5, 1.5).get(0).getInitialHolonomicPose().getY());

    }

    public static Autonomous getInstance(){
        if(autonomous == null){
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    public void defineAutoPaths(){

        // OnePieceBalanceCol3 = PathPlanner.loadPath("1PieceBalanceCol3", 0.5, 0.5);
        // OnePieceBalanceCol7 = PathPlanner.loadPath("1PieceBalanceCol7", 0.5, 0.5);
        // OnePieceSkedaddleMid = PathPlanner.loadPath("1PieceSkedaddleMid", 0.5, 0.5);
        // OnePiecePrepareCol9 = PathPlanner.loadPath("1PiecePrepareCol9", 0.5, 0.5);

        // TwoPieceFree = PathPlanner.loadPath("2PieceFree", 0.5, 0.5);
        // TwoPieceBump = PathPlanner.loadPath("2PieceBump", 0.5, 0.5);
        // TwoPieceFreeBalance = PathPlanner.loadPath("2PieceFreeBalance", 0.5, 0.5);
        // TwoPieceFreePrepareCol1 = PathPlanner.loadPath("2PieceFreePrepareCol1", 0.5, 0.5);
        // TwoPieceBumpPrepareCol9 = PathPlanner.loadPath("2PieceBumpPrepareCol9", 0.5, 0.5);
        // ThreePieceFree = PathPlanner.loadPath("3PieceFree", 0.5, 0.5);
    }


    public void setupAutoRoutines(){

        //Mock competition paths
        // autoRoutines.put("1 Piece Balance Column 3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol3", 0.5, 0.5)));
        // autoRoutines.put("1 Piece Balance Column 7", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol7", 0.5, 0.5)));
        // autoRoutines.put("1 Piece Skedaddle Mid", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceSkeddadleMid", 0.5, 0.5)));
        // autoRoutines.put("1 Piece Prepare Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PiecePrepareCol9", 0.5, 0.5)));

        // autoRoutines.put("2 Piece Free", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceFree", 0.5, 0.5)));
        // autoRoutines.put("2 Piece Bump", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBump", 0.5, 0.5)));
        // autoRoutines.put("2 Piece Free Balance", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceFreeBalance", 0.5, 0.5)));
        // autoRoutines.put("2 Piece Free Prepare Column 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceFreePrepareCol1", 0.5, 0.5)));
        // autoRoutines.put("2 Piece Bump Prepare Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBumpPrepareCol9", 0.5, 0.5)));
        // autoRoutines.put("3 Piece Free", autoBuilder.fullAuto(PathPlanner.loadPathGroup("3PieceFree", 0.5, 0.5)));

        //test paths
        autoRoutines.put("testPath", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath", 0.5, 0.5)));
        autoRoutines.put("testPath1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath1", 0.5, 0.5)));
        autoRoutines.put("testPath2", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath2", 0.5, 0.5)));
        autoRoutines.put("testPath3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath3", 0.5, 0.5)));
        autoRoutines.put("testPath4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath4", 0.5, 0.5)));
    }   


    public void setupAutoSelector(){
        Enumeration<String> e = autoRoutines.keys();

        while (e.hasMoreElements()) {
            String autoRoutineName = e.nextElement();
            autoRoutineSelector.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        }

        SmartDashboard.putData("Auto Routines", autoRoutineSelector);
    }

    private void setFlipped(){ //used only in auto
        drivetrain.setFlipped();
    }

    public Command getAutonomousCommand(){
        return autoRoutineSelector.getSelected();
    }

    //Does not reflect for red -- DO NOT CHANGE
    public CommandBase createCommandFromTrajectory(List<PathPlannerTrajectory> trajectory){
        return autoBuilder.fullAuto(trajectory);
    }

    public double getAngleOffsetFromAuto(){
        return 180.0; // NEED TO IMPLEMENT BY RETURNING INITIAL POSE FROM AUTO. Defaulting to 180 degrees.
    }

}
