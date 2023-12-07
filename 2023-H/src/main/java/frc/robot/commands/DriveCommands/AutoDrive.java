package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriverOI;
import frc.robot.utils.DriverOI.DPadDirection;

public class AutoDrive extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private DriverOI driverOI = DriverOI.getInstance();
  private Translation2d Translation;
  private double Rotation;


  /** Creates a new SwerveDriveCommand. */
  public AutoDrive(Translation2d t, double r) {
    // Use addRequirements() here to declare subsystem dependencies.
    Translation=t;
    Rotation=r;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.drive(Translation, Rotation, true, new Translation2d(0, 0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}