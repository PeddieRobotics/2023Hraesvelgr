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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private SwerveAutoBuilder autoBuilder;

    // Paths
    private PathPlannerTrajectory OneMeterStraight, SwerveTest, SwerveTestRed;

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        
        // Setup sendable chooser
        autoRoutines = new Hashtable<String, Command>();
        autoRoutineSelector = new SendableChooser<Command>();

        HashMap<String, Command> eventMap = new HashMap<>();

        autoBuilder = new SwerveAutoBuilder(
            drivetrain ::getPose, drivetrain ::resetRobotPoseAndGyro, DriveConstants.kinematics, new PIDConstants(AutoConstants.kPTranslationController, 0, 0), new PIDConstants(AutoConstants.kPThetaController, 0, 0), drivetrain::setSwerveModuleStates, eventMap, true, drivetrain
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
        OneMeterStraight = PathPlanner.loadPath("1MeterStraight", 0.5, 0.5);
        SwerveTest = PathPlanner.loadPath("SwerveTest", 1, 1);
        SwerveTestRed = PathPlanner.loadPath("SwerveTestRed", 0.5, 0.5);


    }


    public void setupAutoRoutines(){
        autoRoutines.put("1 Meter Straight Path", createCommandFromTrajectory(PathPlanner.loadPathGroup("1MeterStraight", 2,2)));
        autoRoutines.put("1 Meter Straight Path Spin", createCommandFromTrajectory(PathPlanner.loadPathGroup("1MeterStraightSpin", 0.5, 0.5)));
        autoRoutines.put("U-path", createCommandFromTrajectory(PathPlanner.loadPathGroup("U-path", 0.5, 0.5)));
        autoRoutines.put("Star", createCommandFromTrajectory(PathPlanner.loadPathGroup("Star", 0.5, 0.5)));
        autoRoutines.put("ShootingStar", createCommandFromTrajectory(PathPlanner.loadPathGroup("ShootingStar", 0.5, 0.5)));
        autoRoutines.put("SwerveTest", createCommandFromTrajectory(PathPlanner.loadPathGroup("SwerveTest", 0.3, 0.3)));
        autoRoutines.put("2 Meter Straight Path", createCommandFromTrajectory(PathPlanner.loadPathGroup("2MeterStraight", 1.5, 1.5)));

        //Mock competition paths
        autoRoutines.put("1 Cone Skedaddle", createCommandFromTrajectory(PathPlanner.loadPathGroup("1ConeSkedaddle", 1,1)));
        autoRoutines.put("1 Cone Free", createCommandFromTrajectory(PathPlanner.loadPathGroup("1ConeFree", 1,1)));
        autoRoutines.put("1 Cone Block", createCommandFromTrajectory(PathPlanner.loadPathGroup("1ConeBlock", 1,1)));
        autoRoutines.put("2 Piece Free", createCommandFromTrajectory(PathPlanner.loadPathGroup("2PieceFree", 1,1)));
        autoRoutines.put("2 Piece Block", createCommandFromTrajectory(PathPlanner.loadPathGroup("2PieceBlock", 1,1)));
        autoRoutines.put("2 Piece Bridge Left", createCommandFromTrajectory(PathPlanner.loadPathGroup("2PieceBridgeL", 1,1)));
        autoRoutines.put("2 Piece Bridge Right", createCommandFromTrajectory(PathPlanner.loadPathGroup("2PieceBridgeR", 1,1)));
        autoRoutines.put("3 Link Scatter Left", createCommandFromTrajectory(PathPlanner.loadPathGroup("3LinkScatterL", 1,1)));
        autoRoutines.put("3 Link Scatter Right", createCommandFromTrajectory(PathPlanner.loadPathGroup("3LinkScatterR", 1,1)));
        autoRoutines.put("254", createCommandFromTrajectory(PathPlanner.loadPathGroup("254", 1,1)));
    }   


    public void setupAutoSelector(){
        Enumeration<String> e = autoRoutines.keys();

        while (e.hasMoreElements()) {
            String autoRoutineName = e.nextElement();
            autoRoutineSelector.addOption(autoRoutineName, autoRoutines.get(autoRoutineName));
        }

        SmartDashboard.putData("Auto Routines", autoRoutineSelector);
    }

    public Command getAutonomousCommand(){
        return autoRoutineSelector.getSelected();
    }

    //Does not reflect for red -- DO NOT CHANGE
    public CommandBase createCommandFromTrajectory(List<PathPlannerTrajectory> trajectory){
        return autoBuilder.fullAuto(trajectory);
    }



}
