// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Hashtable;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveCommands.Target;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Drivetrain;

public class Autonomous extends SubsystemBase {
  private static Autonomous autonomous;
  private final Drivetrain drivetrain;

  private SendableChooser<Command> autoRoutineSelector;
  private Hashtable<String, Command> autoRoutines;
  
  public Autonomous() {
    drivetrain = Drivetrain.getInstance();
  }

    public static Autonomous getInstance() {
      if(autonomous == null) {
        autonomous = new Autonomous();
      }
      return autonomous;
      }

      public void setupAutoRoutines() {
        //TO BE FILLED OUT
    }
    

    public Command returnAutonomousCommand() {
        return autoRoutineSelector.getSelected();
    }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}