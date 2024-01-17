package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Hashtable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/* 
 * TODO: loadPathGroup has been moved to a new function (it appears)
 * TODO: PathConstraints constructor now takes 4 params instead of 2
 * TODO: PathPlanner is removed, there are now PathPlannerPlan, PathPlannerTrajectory, PathPlannerAuto and PathPlannerLogging
 * PathConstraints​(double maxVelocityMps, double maxAccelerationMpsSq, double maxAngularVelocityRps, double maxAngularAccelerationRpsSq)
 */


// import com.pathplanner.lib.PathConstraints; becomes:
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.auto.PIDConstants; becomes:
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
import frc.robot.commands.DriveCommands.ClimbCSGyro;
import frc.robot.commands.DriveCommands.ClimbCSGyroDelta;
import frc.robot.commands.DriveCommands.ClimbCSGyroWithAnglePid;
import frc.robot.commands.DriveCommands.LockDrivetrain;
import frc.robot.commands.DriveCommands.RotateToAngle;
import frc.robot.commands.DriveCommands.StraightenDrivetrain;
import frc.robot.utils.Constants;
import frc.robot.commands.DriveCommands.AutoDrive;

// import frc.robot.utils.CustomAutoBuilder;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Autonomous extends SubsystemBase{
    // Subsystems
    private static Autonomous autonomous;
    private final Drivetrain drivetrain;
    private final Claw claw;
    private final Arm arm;
    //private final Claw claw;

    private static SendableChooser<Command> autoChooser;

    private HashMap<String, Command> eventMap;

    // Auto Builder
    // private CustomAutoBuilder autoBuilder;

    // Paths

    public Autonomous(){
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        claw = Claw.getInstance();

        NamedCommands.registerCommand("ClimbCSFrontSlow", new SequentialCommandGroup(new ClimbCSGyro(0, 1.0, 0.75), new LockDrivetrain()));

        NamedCommands.registerCommand("ClimbCSBackSlow", new SequentialCommandGroup(new ClimbCSGyroDelta(180, 1.0, 0.75), new LockDrivetrain()));

        NamedCommands.registerCommand("TranslateRotate", new AutoDrive(new Translation2d(-.3, 0), 0.5 * Constants.DriveConstants.kMaxAngularSpeed));

        // TODO: tune PIDConstants

        // AutoBuilder.configureHolonomic(
        //     drivetrain::getPose, // Robot pose supplier
        //     drivetrain::resetRobotPoseAndGyro, // Method to reset odometry (will be called if your auto has a starting pose)
        //     drivetrain::getRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //     drivetrain::driveAuton, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        //         new PIDConstants(6, 0.0, 0.0), // Translation PID constants
        //         new PIDConstants(4.5, 0.0, 0.0), // Rotation PID constants
        //         Constants.DriveConstants.kMaxFloorSpeed, // Max module speed, in m/s
        //         Constants.DriveConstants.kBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
        //         new ReplanningConfig() // Default path replanning config. See the API for the options here
        //     ),
        //     drivetrain // Reference to drive subsystem to set requirements
        // );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //setupAutoRoutines();

        SmartDashboard.putBoolean("RunAutonWithoutEvents", false);

        // SmartDashboard.putNumber("auto trans p", AutoConstants.kPTranslationController);
        // SmartDashboard.putNumber("auto trans i", AutoConstants.kITranslationController);
        // SmartDashboard.putNumber("auto trans d", AutoConstants.kDTranslationController);

        // SmartDashboard.putNumber("auto theta p", AutoConstants.kPThetaController);
        // SmartDashboard.putNumber("auto theta i", AutoConstants.kIThetaController);
        // SmartDashboard.putNumber("auto theta d", AutoConstants.kDThetaController);

        SmartDashboard.putNumber("Auto Translation P", AutoConstants.kPTranslationController);
        SmartDashboard.putNumber("Auto Rotation P", AutoConstants.kPThetaController);
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Auto Translation P", AutoConstants.kPTranslationController);
        // SmartDashboard.putNumber("Auto Rotation P", AutoConstants.kPThetaController);
    }

    public void resetAutoBuilderAndPaths(){
    }

    public static Autonomous getInstance(){
        if(autonomous == null){
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    @Deprecated
    public void setupAutoRoutines(){
        /*
         * Competition paths start here
         */

        //DEV COMMENT
        ///*

    }   

    // Used only at the start of autonomous
    private void setFlipped(){ 
        drivetrain.setFlipped();
    }

    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
