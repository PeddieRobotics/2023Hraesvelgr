package frc.robot.subsystems.Autonomous;

import frc.robot.subsystems.Autonomous.Autonomous;
import frc.robot.subsystems.Shuffleboard;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonomousConfig extends SubsystemBase{
    public static AutonomousConfig autonomousConfig;
    private Autonomous autonomous;
    private Shuffleboard shuffleboard;

    
    //Sendable Chooser
    private SendableChooser startingGamePiece, startingColumn, scoringLevel, objectiveOne, objectiveTwo, objectiveThree, objectiveFour;

    private String numberOfGamePieces, balance, description, columnNumber, pathName;

    private boolean inversion;

    public AutonomousConfig() {
        autonomous = Autonomous.getInstance();
        shuffleboard = Shuffleboard.getInstance();

    }

    @Override
    public void periodic() {
    }

    public static AutonomousConfig getInstance(){
        if(autonomousConfig == null){
            autonomousConfig = new AutonomousConfig();
        }
        return autonomousConfig;
    }

    public void configAutonomousTab() {
        
    }
}