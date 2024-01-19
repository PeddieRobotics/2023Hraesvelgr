package frc.robot.commands.DriveCommands;

import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFront;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SourceSideAlign extends Command{
    Drivetrain drivetrain; 
    LimelightFront limelightFront; 

    PIDController pidController; 
    double error; 
    double currentTx; 
    double angleThreshold; 
    double FF; 

    Translation2d translation; 


    public SourceSideAlign(){
        angleThreshold = 1.0; 
        error = 0.0; 
        FF = 0.5; 
        // pidController = new PIDController(0.05, 0.0001, 0);
        pidController = new PIDController(0.02, 0.0, 0.0);
        pidController.enableContinuousInput(-180, 180);

        drivetrain = Drivetrain.getInstance(); 
        limelightFront = LimelightFront.getInstance(); 
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){ 
        currentTx = limelightFront.getTxAverage(); 

        if(currentTx>angleThreshold){
            translation = new Translation2d(0.0, pidController.calculate(currentTx, 0) - FF); 
        }
        else if(currentTx<-angleThreshold){
            translation = new Translation2d(0.0, pidController.calculate(currentTx, 0) + FF); 
        }
        else{
            translation = new Translation2d(0.0, 0.0); 
        }
        drivetrain.drive(translation, 0.0, true, new Translation2d(0, 0));
    }
    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {

    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        currentTx = limelightFront.getTxAverage(); 
        return Math.abs(currentTx) <= angleThreshold; 
    }
}
