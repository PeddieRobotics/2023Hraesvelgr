
package frc.robot.utils;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.DriveCommands.StraightenDrivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class CustomAutoBuilder extends BaseAutoBuilder {
  private Runnable setFlipped;
  private static final double FIELD_HEIGHT_METERS = 8.02;
  private static final double FIELD_WIDTH_METERS = 16.54;

  private final SwerveDriveKinematics kinematics;
  private final PIDConstants translationConstants;
  private final PIDConstants rotationConstants;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final Consumer<ChassisSpeeds> outputChassisSpeeds;
  private final Subsystem[] driveRequirements;

  private final boolean useKinematics;

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param translationConstants PID Constants for the controller that will correct for translation
   *     error
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public CustomAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Runnable setFlipped,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      Map<String, Command> eventMap,
      Subsystem... driveRequirements) {
    this(
        poseSupplier,
        resetPose,
        setFlipped,
        translationConstants,
        rotationConstants,
        outputChassisSpeeds,
        eventMap,
        false,
        driveRequirements);
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param translationConstants PID Constants for the controller that will correct for translation
   *     error
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputModuleStates A function that takes raw output module states from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public CustomAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Runnable setFlipped,
      SwerveDriveKinematics kinematics,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Map<String, Command> eventMap,
      Subsystem... driveRequirements) {
    this(
        poseSupplier,
        resetPose,
        setFlipped,
        kinematics,
        translationConstants,
        rotationConstants,
        outputModuleStates,
        eventMap,
        false,
        driveRequirements);
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param translationConstants PID Constants for the controller that will correct for translation
   *     error
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputChassisSpeeds A function that takes the output ChassisSpeeds from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public CustomAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Runnable setFlipped,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      Map<String, Command> eventMap,
      boolean useAllianceColor,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);

    this.kinematics = null;
    this.translationConstants = translationConstants;
    this.rotationConstants = rotationConstants;
    this.outputModuleStates = null;
    this.outputChassisSpeeds = outputChassisSpeeds;
    this.driveRequirements = driveRequirements;

    this.useKinematics = false;
    this.setFlipped=setFlipped;
  }

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPSwerveControllerCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param translationConstants PID Constants for the controller that will correct for translation
   *     error
   * @param rotationConstants PID Constants for the controller that will correct for rotation error
   * @param outputModuleStates A function that takes raw output module states from path following
   *     commands
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public CustomAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Runnable setFlipped,
      SwerveDriveKinematics kinematics,
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Map<String, Command> eventMap,
      boolean useAllianceColor,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);

    this.kinematics = kinematics;
    this.translationConstants = translationConstants;
    this.rotationConstants = rotationConstants;
    this.outputModuleStates = outputModuleStates;
    this.outputChassisSpeeds = null;
    this.driveRequirements = driveRequirements;

    this.useKinematics = true;
    this.setFlipped=setFlipped;
  }

  @Override
  public CommandBase followPath(PathPlannerTrajectory trajectory) {
    if (useKinematics) {
      return new PPSwerveControllerCommand(
          trajectory,
          poseSupplier,
          kinematics,
          pidControllerFromConstants(translationConstants),
          pidControllerFromConstants(translationConstants),
          pidControllerFromConstants(rotationConstants),
          outputModuleStates,
          useAllianceColor,
          driveRequirements);
    } else {
      return new PPSwerveControllerCommand(
          trajectory,
          poseSupplier,
          pidControllerFromConstants(translationConstants),
          pidControllerFromConstants(translationConstants),
          pidControllerFromConstants(rotationConstants),
          outputChassisSpeeds,
          useAllianceColor,
          driveRequirements);
    }
  }



  @Override
    public CommandBase fullAuto(List<PathPlannerTrajectory> pathGroup) {
        List<CommandBase> commands = new ArrayList<>();
    
        commands.add(resetPose(pathGroup.get(0)));
        commands.add(new InstantCommand(setFlipped));
    
        for (PathPlannerTrajectory traj : pathGroup) {
          commands.add(stopEventGroup(traj.getStartStopEvent()));
          commands.add(followPathWithEvents(traj));
        }
    
        commands.add(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));

        return new ConditionalCommand(new SequentialCommandGroup(
                                        resetPose(pathGroup.get(0)),
                                        new StraightenDrivetrain(),
                                        followPathGroup(pathGroup)),
                                      Commands.sequence(commands.toArray(CommandBase[]::new)), 
                                      () -> SmartDashboard.getBoolean("RunAutonWithoutEvents", false));
  }


  @Override
  public CommandBase resetPose(PathPlannerTrajectory trajectory) {
    if (drivetrainType == DrivetrainType.HOLONOMIC) {
      return Commands.runOnce(
          () -> {
            PathPlannerTrajectory.PathPlannerState initialState = trajectory.getInitialState();
            if (useAllianceColor) {
              initialState =
              PathPlannerTrajectory.transformStateForAlliance(
                      initialState, DriverStation.getAlliance());
            }

            resetPose.accept(
                new Pose2d(
                    initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
          });
    } else {
      return Commands.runOnce(
          () -> {
            PathPlannerTrajectory.PathPlannerState initialState = trajectory.getInitialState();
            if (useAllianceColor) {
              initialState =
              PathPlannerTrajectory.transformStateForAlliance(
                      initialState, DriverStation.getAlliance());
            }

            resetPose.accept(initialState.poseMeters);
          });
    }
  }

  // public static PathPlannerState transformStateForAlliance(
  //     PathPlannerState state, DriverStation.Alliance alliance) {
  //   if (alliance == DriverStation.Alliance.Red) {
  //     // Create a new state so that we don't overwrite the original
  //     PathPlannerState transformedState = new PathPlannerState();
  //     transformedState = PathPlannerTrajectory.transformStateForAlliance(state, alliance);

  //     Translation2d transformedTranslation =
  //         new Translation2d(FIELD_WIDTH_METERS-state.poseMeters.getX(), state.poseMeters.getY());

  //     transformedState.poseMeters = new Pose2d(transformedTranslation, state.poseMeters.getRotation());

  //     return transformedState;
  //   } else {
  //     return state;
  //   }
  // }
}