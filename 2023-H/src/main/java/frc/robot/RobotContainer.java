// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.utils.UpdateLogs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain;
  private final OI oi;
  private Command autoCommand;
  private final Intake intake;
  private final Arm arm;
  //private final UpdateLogs logs;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    // configureButtonBindings();

    drivetrain = Drivetrain.getInstance();
    oi = OI.getInstance(); // Make sure OI gets initialized here, this should be the first call to
                           // getInstance()
    // Set up a default command to ensure the robot drives by default
    drivetrain.setDefaultCommand(new SwerveDriveCommand());
    //logs = UpdateLogs.getInstance();
    intake = Intake.getInstance();
    arm = Arm.getInstance();

    SmartDashboard.putNumber("Drive P", 0.0);
    SmartDashboard.putNumber("Drive I", 0.0);
    SmartDashboard.putNumber("Drive D", 0.0);
    SmartDashboard.putNumber("Drive FF", 0.0);
    SmartDashboard.putNumber("Angle P", 0.0);
    SmartDashboard.putNumber("Angle I", 0.0);
    SmartDashboard.putNumber("Angle D", 0.0);
    SmartDashboard.putNumber("Angle FF", 0.0);

    SmartDashboard.putBoolean("Command Activated?", false);
    SmartDashboard.putBoolean("Aiming?", false);
    SmartDashboard.putBoolean("Shooting?", false);
    SmartDashboard.putBoolean("Intake Deployed?", false);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  // private void configureButtonBindings() {}

  // /**
  // * Use this to pass the autonomous command to the main {@link Robot} class.
  // *
  // * @return the command to run in autonomous
  // */
  public Command getAutonomousCommand() {
    drivetrain.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    return new WaitCommand(1);
  }
 
  public void setupSmartDashboardTestMode() {
    SmartDashboard.putNumber("FL Drive", 0.0);
    SmartDashboard.putNumber("FL Angle", 0.0);
    SmartDashboard.putNumber("FR Drive", 0.0);
    SmartDashboard.putNumber("FR Angle", 0.0);
    SmartDashboard.putNumber("BL Drive", 0.0);
    SmartDashboard.putNumber("BL Angle", 0.0);
    SmartDashboard.putNumber("BR Drive", 0.0);
    SmartDashboard.putNumber("BR Angle", 0.0);
  }

  public void testAllSystems() {
    drivetrain.getFrontLeftSwerveModule().setDriveMotor(SmartDashboard.getNumber("FL Drive", 0.0));
    drivetrain.getFrontLeftSwerveModule().setAngleMotor(SmartDashboard.getNumber("FL Angle", 0.0));
    drivetrain.getFrontRightSwerveModule().setDriveMotor(SmartDashboard.getNumber("FR Drive", 0.0));
    drivetrain.getFrontRightSwerveModule().setAngleMotor(SmartDashboard.getNumber("FR Angle", 0.0));
    drivetrain.getBackLeftSwerveModule().setDriveMotor(SmartDashboard.getNumber("BL Drive", 0.0));
    drivetrain.getBackLeftSwerveModule().setAngleMotor(SmartDashboard.getNumber("BL Angle", 0.0));
    drivetrain.getBackRightSwerveModule().setDriveMotor(SmartDashboard.getNumber("BR Drive", 0.0));
    drivetrain.getBackRightSwerveModule().setAngleMotor(SmartDashboard.getNumber("BR Angle", 0.0));

    intake.updateIntakeFromDashboard();
  }

  public void reportAllSwerveModuleStates() {
    SmartDashboard.putNumber("FL Drive State", drivetrain.getSwerveModules()[0].getCurrentState().speedMetersPerSecond);
    SmartDashboard.putNumber("FL Angle State", drivetrain.getSwerveModules()[0].getCurrentState().angle.getRadians());
    SmartDashboard.putNumber("FR Drive State", drivetrain.getSwerveModules()[1].getCurrentState().speedMetersPerSecond);
    SmartDashboard.putNumber("FR Angle State", drivetrain.getSwerveModules()[1].getCurrentState().angle.getRadians());
    SmartDashboard.putNumber("BL Drive State", drivetrain.getSwerveModules()[2].getCurrentState().speedMetersPerSecond);
    SmartDashboard.putNumber("BL Angle State", drivetrain.getSwerveModules()[2].getCurrentState().angle.getRadians());
    SmartDashboard.putNumber("BR Drive State", drivetrain.getSwerveModules()[3].getCurrentState().speedMetersPerSecond);
    SmartDashboard.putNumber("BR Angle State", drivetrain.getSwerveModules()[3].getCurrentState().angle.getRadians());

  }

  public void resetGyro() {
    drivetrain.resetGyro();
  }

  public void resetRobotPose() {
    drivetrain.resetRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  public void setupAngleOffsetFromAuto(double target) {
    drivetrain.setTeleOpAngleOffset(target);
  }

  // public void startLogging(){
  //   logs.startLogging();
  // }

  // public void stopLogging(){
  //   logs.stopLogging();;
  // }

}