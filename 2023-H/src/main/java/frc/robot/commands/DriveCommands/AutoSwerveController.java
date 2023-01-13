package frc.robot.commands.DriveCommands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class AutoSwerveController extends CommandBase {

    private final Timer timer;
    private final Trajectory trajectory;

    //rename, remove m_?
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;

    public AutoSwerveController(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
            PIDController xController, PIDController yController, ProfiledPIDController thetaController,
            Subsystem... requirements) {
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
        m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
        m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");

        m_controller = new HolonomicDriveController(
                requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
                requireNonNullParam(yController, "xController", "SwerveControllerCommand"),
                requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));

        m_controller.setTolerance(new Pose2d(0.5, 0.5, new Rotation2d(0.25)));

        timer = new Timer();

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(curTime);
        SmartDashboard.putNumber("predicted autonomous angle", desiredState.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("predicted autonomous x", desiredState.poseMeters.getX());
        SmartDashboard.putNumber("predicted autonomous y", desiredState.poseMeters.getY());
        var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
        var moduleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
        Drivetrain.getInstance().setSwerveModuleStates(moduleStates);
        // Trajectory.State desiredState = trajectory.sample(curTime);
        // var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState,
        // desiredState.);
        // Drivetrain.getInstance().setSwerveModuleStates();
    }
}