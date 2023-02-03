package frc.robot.subsystems;

import java.util.Enumeration;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoCommands.TestPath;
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

    private PathPlannerTrajectory OnePieceSkeddadle, OnePieceSkeddadlecol7, OnePieceSkeddadlecol3, OnePieceSkeddadlecol6, OnePieceSkeddadlecol4, OnePieceSkeddadleMid, OnePieceCol3, OnePieceCol7, OnePieceBacksideBridgeCol1, OnePieceBacksideBridgeCol2, OnePieceBacksideBridgeCol8, OnePieceBacksideBridgeCol9;

    private PathPlannerTrajectory TwoPieceFree, TwoPieceBump, TwoPieceFreeBalance_Part2, TwoPieceBumpBalance_Part1, TwoPieceBumpBalance_Part2, TwoPieceFreePrepareCol1, TwoPieceBumpPrepareCol9;

    private PathPlannerTrajectory ThreePieceFree, ThreePieceBumpBalance_Part1, ThreePieceBumpBalance_Part2, ThreePieceBumpBalance_Part3;

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        
        // Setup sendable chooser
        autoRoutines = new Hashtable<String, Command>();
        autoRoutineSelector = new SendableChooser<Command>();

        HashMap<String, Command> eventMap = new HashMap<>();

        autoBuilder = new SwerveAutoBuilder(
            drivetrain ::getPose, drivetrain ::resetRobotPosition, DriveConstants.kinematics, new PIDConstants(AutoConstants.kPTranslationController, 0, 0), new PIDConstants(AutoConstants.kPThetaController, 0, 0), drivetrain::setSwerveModuleStates, eventMap, true, drivetrain
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
        //Test Paths
        OneMeterStraight = PathPlanner.loadPath("1MeterStraight", 0.5, 0.5);
        SwerveTest = PathPlanner.loadPath("SwerveTest", 1, 1);
        SwerveTestRed = PathPlanner.loadPath("SwerveTestRed", 0.5, 0.5);

        //Comp paths

        autoRoutines.put("OnePieceSkeddadle", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceSkeddadle", 0.5, 0.5))));
        autoRoutines.put("OnePieceSkeddadlecol7", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceSkeddadlecol7", 0.5, 0.5))));
        autoRoutines.put("OnePieceSkeddadlecol3", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceSkeddadlecol3", 0.5, 0.5))));
        autoRoutines.put("OnePieceSkeddadlecol6", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceSkeddadlecol6", 0.5, 0.5))));
        autoRoutines.put("OnePieceSkeddadlecol4", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceSkeddadlecol4", 0.5, 0.5))));
        autoRoutines.put("OnePieceSkeddadleMid", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceSkeddadleMid", 0.5, 0.5))));

        autoRoutines.put("OnePieceBridgeCol3", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceBridgeCol3", 0.5, 0.5))));
        autoRoutines.put("OnePieceBridgeCol7", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceBridgeCol7", 0.5, 0.5))));
        autoRoutines.put("OnePieceBacksideBridgeCol1", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceBacksideBridgeCol1", 0.5, 0.5))));
        autoRoutines.put("OnePieceBacksideBridgeCol2", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceBacksideBridgeCol2", 0.5, 0.5))));
        autoRoutines.put("OnePieceBacksideBridgeCol8", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceBacksideBridgeCol8", 0.5, 0.5))));
        autoRoutines.put("OnePieceBacksideBridgeCol9", autoBuilder.fullAuto((PathPlanner.loadPathGroup("1PieceBacksideBridgeCol9", 0.5, 0.5))));

        autoRoutines.put("TwoPieceFree", autoBuilder.fullAuto((PathPlanner.loadPathGroup("2PieceFree", 0.5, 0.5))));
        autoRoutines.put("TwoPieceFreeBalance_Part2", autoBuilder.fullAuto((PathPlanner.loadPathGroup("2PieceFreeBalance_Part1", 0.5, 0.5))));
        autoRoutines.put("TwoPieceFreePrepareCol1", autoBuilder.fullAuto((PathPlanner.loadPathGroup("2PieceFreePrepareCol1", 0.5, 0.5))));
        autoRoutines.put("TwoPieceBump", autoBuilder.fullAuto((PathPlanner.loadPathGroup("2PieceBump", 0.5, 0.5))));
        autoRoutines.put("TwoPieceBumpBalance_Part1", autoBuilder.fullAuto((PathPlanner.loadPathGroup("2PieceBumpBalance_Part1", 0.5, 0.5))));
        autoRoutines.put("TwoPieceBumpBalance_Part2", autoBuilder.fullAuto((PathPlanner.loadPathGroup("2PieceBumpBalance_Part1", 0.5, 0.5))));
        autoRoutines.put("TwoPieceBumpPrepareCol9", autoBuilder.fullAuto((PathPlanner.loadPathGroup("2PieceFreePrepareCol9", 0.5, 0.5))));

        autoRoutines.put("ThreePieceFree", autoBuilder.fullAuto((PathPlanner.loadPathGroup("3PieceFree", 0.5, 0.5))));
        autoRoutines.put("ThreePieceBumpBalance_Part1", autoBuilder.fullAuto((PathPlanner.loadPathGroup("3PieceBumpBalance_Part1", 0.5, 0.5))));
        autoRoutines.put("ThreePieceBumpBalance_Part2", autoBuilder.fullAuto((PathPlanner.loadPathGroup("3PieceBumpBalance_Part2", 0.5, 0.5))));
        autoRoutines.put("ThreePieceBumpBalance_Part3", autoBuilder.fullAuto((PathPlanner.loadPathGroup("3PieceBumpBalance_Part3", 0.5, 0.5))));

    }


    public void setupAutoRoutines(){
        //
        autoRoutines.put("1 Meter Straight Path", createCommandFromTrajectory(PathPlanner.loadPathGroup("1MeterStraight", 2,2)));
        autoRoutines.put("1 Meter Straight Path Spin", createCommandFromTrajectory(PathPlanner.loadPathGroup("1MeterStraightSpin", 0.5, 0.5)));
        autoRoutines.put("U-path", createCommandFromTrajectory(PathPlanner.loadPathGroup("U-path", 0.5, 0.5)));
        autoRoutines.put("Star", createCommandFromTrajectory(PathPlanner.loadPathGroup("Star", 0.5, 0.5)));
        autoRoutines.put("ShootingStar", createCommandFromTrajectory(PathPlanner.loadPathGroup("ShootingStar", 0.5, 0.5)));
        autoRoutines.put("SwerveTest", createCommandFromTrajectory(PathPlanner.loadPathGroup("SwerveTest", 0.3, 0.3)));
        autoRoutines.put("2 Meter Straight Path", createCommandFromTrajectory(PathPlanner.loadPathGroup("2MeterStraight", 1.5, 1.5)));

        //Mock competition paths
        autoRoutines.put("One Piece Skeddadle", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceSkeddadle", 0.5, 0.5)));
        autoRoutines.put("One Piece Skeddadle Column 7", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceSkeddadlecol7", 0.5, 0.5)));
        autoRoutines.put("One Piece Skeddadle Column 3", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceSkeddadlecol3", 0.5, 0.5)));
        autoRoutines.put("One Piece Skeddadle Column 6", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceSkeddadlecol6", 0.5, 0.5)));
        autoRoutines.put("One Piece Skeddadle Column 4", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceSkeddadlecol4", 0.5, 0.5)));
        autoRoutines.put("One Piece Skeddadle Middle", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceSkeddadleMid", 0.5, 0.5)));
        autoRoutines.put("One Piece Bridge Column 7", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceBridgeCol7", 0.5, 0.5)));
        autoRoutines.put("One Piece Bridge Column 3", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceBridgeCol3", 0.5, 0.5)));
        autoRoutines.put("One Piece Backside Bridge Column 1", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceBacksideBridgeCol1", 0.5, 0.5)));
        autoRoutines.put("One Piece Backside Bridge Column 2", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceBacksideBridgeCol2", 0.5, 0.5)));
        autoRoutines.put("One Piece Backside Bridge Column 8", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceBacksideBridgeCol8", 0.5, 0.5)));
        autoRoutines.put("One Piece Backside Bridge Column 9", createCommandFromTrajectory(PathPlanner.loadPathGroup("1PieceBacksideBridgeCol9", 0.5, 0.5)));

        autoRoutines.put("Two Piece Free", createCommandFromTrajectory(PathPlanner.loadPathGroup("2PieceFree", 0.5, 0.5)));
        //
        autoRoutines.put("Two Piece Free Prepare Column 1", createCommandFromTrajectory(PathPlanner.loadPathGroup("2PieceFreePrepareCol1", 0.5, 0.5)));
        autoRoutines.put("Two Piece Bump", createCommandFromTrajectory(PathPlanner.loadPathGroup("2PieceBump", 0.5, 0.5)));
        //
        autoRoutines.put("Two Piece Bump Prepare Column 9", createCommandFromTrajectory(PathPlanner.loadPathGroup("2PieceBumpPrepareCol9", 0.5, 0.5)));

        autoRoutines.put("Three Piece Free", createCommandFromTrajectory(PathPlanner.loadPathGroup("3PieceFree", 0.5, 0.5)));
        //

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
