// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.OI;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
// import frc.robot.utils.UpdateLogs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Claw;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain;
  private final OI oi;
  private Command autoCommand;
  private final Claw intake;
  private final Arm arm;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = Drivetrain.getInstance();
    oi = OI.getInstance();
    drivetrain.setDefaultCommand(new SwerveDriveCommand());
    intake = Claw.getInstance();
    arm = Arm.getInstance();
  }

  public Command getAutonomousCommand() {
    drivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    return new WaitCommand(1);
  }

  public void resetRobotPosition() {
    drivetrain.resetGyro();
    drivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  public void setupAngleOffsetFromAuto(double target) {
    drivetrain.setTeleOpAngleOffset(target);
  }

  // public void startLogging(){
  // logs.startLogging();
  // }

  // public void stopLogging(){
  // logs.stopLogging();;
  // }

}