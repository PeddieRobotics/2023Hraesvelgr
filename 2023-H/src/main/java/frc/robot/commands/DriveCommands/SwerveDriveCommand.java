package frc.robot.commands.DriveCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.OI;
import frc.robot.utils.OI.DPadDirection;

public class SwerveDriveCommand extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private OI oi = OI.getInstance();


  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies. 
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d position;

    if(oi.getDriverDPadInput() != DPadDirection.NONE){
      position = oi.getCardinalDirection();
    }
    else{
      position = oi.getSwerveTranslation();
    }

    double rotation = -oi.getRotation();

    Translation2d centerOfRotation = oi.getCenterOfRotation();

    drivetrain.drive(position, rotation, true, centerOfRotation);

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