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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.SetCompactFloorConePose;
import frc.robot.commands.ArmCommands.SetCompactFloorCubePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorConePose;
import frc.robot.commands.ArmCommands.SetExtendedFloorCubePose;
import frc.robot.commands.ArmCommands.SetL3TransitoryPose;
import frc.robot.commands.ArmCommands.SetLevelThreeConePose;
import frc.robot.commands.ArmCommands.SetLevelThreeConePoseInAuto;
import frc.robot.commands.ArmCommands.SetLevelThreeCubePose;
import frc.robot.commands.ArmCommands.SetStowedPose;
import frc.robot.commands.AutoCommands.AutonAlign;
import frc.robot.commands.AutoCommands.ClimbCSAprilTag;
import frc.robot.commands.ClawCommands.EjectGamepiece;
import frc.robot.commands.ClawCommands.IntakeCone;
import frc.robot.commands.ClawCommands.IntakeCube;
import frc.robot.commands.DriveCommands.LockDrivetrain;
import frc.robot.commands.LimelightCommands.SetPipe;
import frc.robot.commands.LimelightCommands.SetPipeType;
import frc.robot.utils.CustomAutoBuilder;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.WristConstants;

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
        // eventMap.put("dummyDashboardCommand", new DummyDashboardCommand());
        // eventMap.put("DeployIntake", new DummyDeployIntakeCommand());
        // eventMap.put("AimAndShoot", new DummyAimAndShootCommand()); //eventMap.put("AimAndShoot", new DummyAimAndShootCommand());
        for(int i=1;i<=9;i++){
            eventMap.put("AlignCol"+i, new AutonAlign(i));
            eventMap.put("pipe"+i, new SetPipe(i));
            eventMap.put("pipeType"+i, new SetPipeType(i));
        }
        eventMap.put("pipe0", new SetPipe(0));
        eventMap.put("stow", new SetStowedPose());
        eventMap.put("eject", new EjectGamepiece());
        eventMap.put("lock", new LockDrivetrain());
        eventMap.put("IntakeCone", new ParallelRaceGroup(new WaitCommand(4),new IntakeCone()));
        eventMap.put("IntakeCube", new ParallelRaceGroup(new WaitCommand(4),new IntakeCube()));

        eventMap.put("ConeL3", new SequentialCommandGroup(new SetLevelThreeConePoseInAuto(), new SetL3TransitoryPose()));
        eventMap.put("CubeL3", new EjectGamepiece()); //new SetStowedPose()));
        eventMap.put("CubeL3Pose", new SetLevelThreeCubePose()); //new SetStowedPose()));
        eventMap.put("ConeL3Pose", new SetLevelThreeConePose());
    

        eventMap.put("IntakeConePose", new SetCompactFloorConePose());
        eventMap.put("IntakeCubePose", new SetCompactFloorCubePose());
        // eventMap.put("StartIntakingCube", new SequentialCommandGroup( new ParallelRaceGroup( new IntakeCube(), new WaitCommand(4)),new SetStowedPose()));
        // eventMap.put("BalanceFront", new ClimbCSAprilTag(1.5, drivetrain.getHeading()));
        // eventMap.put("BalanceBack", new ClimbCSAprilTag(1.5, drivetrain.getHeading()));
        eventMap.put("BalanceNearFrontLL", new ClimbCSAprilTag(1.25, 180, true, true));
        eventMap.put("BalanceFarFrontLL", new ClimbCSAprilTag(1.25, 0, false, true));
        eventMap.put("BalanceNearBackLL", new ClimbCSAprilTag(1.25, 180, true, false));
        eventMap.put("BalanceFarBackLL", new ClimbCSAprilTag(1.25, 0, false, false));


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

        //defineAutoPaths();
        setupAutoRoutines();
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
    }


    public void setupAutoRoutines(){

        //Mock competition paths
        autoRoutines.put("1 Piece Balance Back Column 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceBackCol1", 0.5, 0.5)));
        autoRoutines.put("1 Piece Balance Back Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceBackCol9", 0.5, 0.5)));
        autoRoutines.put("1 Piece Balance Column 3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol3", 0.5, 0.5)));
        autoRoutines.put("1 Piece Balance Column 4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol4", 0.5, 0.5)));
        autoRoutines.put("1 Piece Balance Column 6", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol6", 0.5, 0.5)));
        autoRoutines.put("1 Piece Balance Column 7", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol7", 0.5, 0.5)));
        autoRoutines.put("1 Piece Balance Column 5", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceBalanceCol5", 0.5, 0.5)));
        autoRoutines.put("1 Piece Skedaddle Column 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceSkedaddleCol1", 0.5, 0.5)));
        autoRoutines.put("1 Piece Skedaddle Column 9", autoBuilder.fullAuto(PathPlanner.loadPathGroup("1PieceSkedaddleCol9", 0.5, 0.5)));


        autoRoutines.put("2 Piece Balance Bump", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBalanceBump", 0.5, 0.5)));
        autoRoutines.put("2 Piece Balance Top", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBalanceTop", 0.5, 0.5)));
        autoRoutines.put("2 Piece Bump", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceBump", 0.5, 0.5)));
        autoRoutines.put("2 Piece Prepare Top", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PiecePrepareTop", 0.5, 0.5)));
        autoRoutines.put("2PieceTop", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2PieceTop", 0.5, 0.5)));

        // autoRoutines.put("3 Piece Balance Top", autobuilder.fullAuto(PathPlanner.loadPathGroup("3PieceBalanceTop", 0.5, 0.5)));
        // autoRoutines.put("3 Piece Top", autobuilder.fullAuto(PathPlanner.loadPathGroup("3PieceTop", 0.5, 0.5)))

        // //test paths
        // int i=10;
        // while(i-->1){
        //     autoRoutines.put("TestPath"+i, autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath"+i, 1, 1)));
        // }
        // autoRoutines.put("TestPath", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath", 0.5, 0.5)));
        // autoRoutines.put("testPath1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath1", 0.5, 0.5)));
        // autoRoutines.put("testPath2", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath2", 0.5, 0.5)));
        // autoRoutines.put("testPath3", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath3", 0.5, 0.5)));
        // autoRoutines.put("testPath4", autoBuilder.fullAuto(PathPlanner.loadPathGroup("TestPath4", 0.5, 0.5)));

        //Testing auto paths 
        autoRoutines.put("Testing Auto Path 1", autoBuilder.fullAuto(PathPlanner.loadPathGroup("testingautopart1", 1.5, 1.5)));
        autoRoutines.put("Testing Auto Path 2", autoBuilder.fullAuto(PathPlanner.loadPathGroup("testingautopart2", 1.5, 1.5)));
    }   

    private void setFlipped(){ //used only in auto
        drivetrain.setFlipped();
    }

    //Does not reflect for red -- DO NOT CHANGE
    public CommandBase createCommandFromTrajectory(List<PathPlannerTrajectory> trajectory){
        return autoBuilder.fullAuto(trajectory);
    }

    public double getAngleOffsetFromAuto(){
        return 180.0; // NEED TO IMPLEMENT BY RETURNING INITIAL POSE FROM AUTO. Defaulting to 180 degrees.
    }

    public Hashtable<String, Command> getAutoRoutines() {
        return autoRoutines;
    }

}
