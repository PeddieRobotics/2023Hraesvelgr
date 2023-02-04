package frc.robot.subsystems.Autonomous;

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
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Autonomous extends SubsystemBase{
    // Subsystems
    private static Autonomous autonomous;
    private final Drivetrain drivetrain;

    // Configured autonomous command
    private String autonomousDescription;
    private CommandBase autonomousRoutine;

    // Auto Builder
    private SwerveAutoBuilder autoBuilder;

    // Paths
    private PathPlannerTrajectory OneMeterStraight, SwerveTest, SwerveTestRed;

    private PathPlannerTrajectory OnePieceSkeddadle, OnePieceSkeddadlecol7, OnePieceSkeddadlecol3, OnePieceSkeddadlecol6, OnePieceSkeddadlecol4, OnePieceSkeddadleMid, OnePieceCol3, OnePieceCol7, OnePieceBacksideBridgeCol1, OnePieceBacksideBridgeCol2, OnePieceBacksideBridgeCol8, OnePieceBacksideBridgeCol9;

    private PathPlannerTrajectory TwoPieceFree, TwoPieceBump, TwoPieceFreeBalance, TwoPieceBumpBalance, TwoPieceFreePrepareCol1, TwoPieceBumpPrepareCol9, TwoCenterPieceBalance;

    private PathPlannerTrajectory ThreePieceFree, ThreePieceBumpBalance, ThreePieceEasyLinkBalance;

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        
        // Setup sendable chooser
        autoRoutines = new Hashtable<String, Command>();
        autoRoutineSelector = new SendableChooser<Command>();

        HashMap<String, Command> eventMap = new HashMap<>();

        autoBuilder = new SwerveAutoBuilder(
            drivetrain ::getPose, drivetrain ::resetRobotPosition, DriveConstants.kinematics, new PIDConstants(AutoConstants.kPTranslationController, 0, 0), new PIDConstants(AutoConstants.kPThetaController, 0, 0), drivetrain::setSwerveModuleStates, eventMap, true, drivetrain
        );

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

    //Modified methods

    public void configureAutoRoutine(String pathName, String description, double velocity, double acceleration){
        autonomousDescription = description;
        autonomousRoutine = autoBuilder.fullAuto((PathPlanner.loadPathGroup(pathName, velocity, acceleration)));
    }

    //marks the end of modified methods 2/4/2023 DO NOT MERGE WITH DEV UNTIL WE HAVE RESOLVED EVERYTHING NEEDED FOR THE SHUFFLEBOARD INTERFACE


    public Command getAutonomousCommand(){
        return autonomousRoutine;
    }

    //Does not reflect for red -- DO NOT CHANGE
    public CommandBase createCommandFromTrajectory(List<PathPlannerTrajectory> trajectory){
        return autoBuilder.fullAuto(trajectory);
    }



}