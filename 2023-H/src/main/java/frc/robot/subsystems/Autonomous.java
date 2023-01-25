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

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        
        // Setup sendable chooser
        autoRoutines = new Hashtable<String, Command>();
        autoRoutineSelector = new SendableChooser<Command>();

        HashMap<String, Command> eventMap = new HashMap<>();

        autoBuilder = new SwerveAutoBuilder(
            drivetrain ::getPose, drivetrain ::resetRobotPosition, DriveConstants.kinematics, new PIDConstants(AutoConstants.kPTranslationController, 0, 0), new PIDConstants(AutoConstants.kPThetaController, 0, 0), drivetrain::setSwerveModuleStates, eventMap, drivetrain
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
        

    }


    public void setupAutoRoutines(){
        autoRoutines.put("1 Meter Straight Path", createCommandFromTrajectory(PathPlanner.loadPathGroup("1MeterStraight", 2,2)));
        autoRoutines.put("1 Meter Straight Path Spin", createCommandFromTrajectory(PathPlanner.loadPathGroup("1MeterStraightSpin", 0.5, 0.5)));
        autoRoutines.put("SwerveTest", createCommandFromTrajectory(PathPlanner.loadPathGroup("SwerveTest", 0.3, 0.3)));
        autoRoutines.put("2 Meter Straight Path", createCommandFromTrajectory(PathPlanner.loadPathGroup("2MeterStraight", 1.5, 1.5)));
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

    public CommandBase createCommandFromTrajectory(List<PathPlannerTrajectory> trajectory){
        return autoBuilder.fullAuto(trajectory);
    }



}
