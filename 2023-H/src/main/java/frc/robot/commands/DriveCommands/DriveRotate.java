package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveRotate extends Command{
    private Drivetrain drivetrain;
    private double initialTime;

    public DriveRotate(){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        initialTime=Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        double t = Timer.getFPGATimestamp();
        double tx=0,ty=0,rot=0;
        if(t-initialTime<1){
            ty=0;
            tx=1;
            rot=0;
        } else {
            tx=Math.sqrt(Math.sqrt(t));
            rot=1;
        }

        drivetrain.drive(new Translation2d(tx,ty),rot,true,new Translation2d());
    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        return false;
    }
}

