package frc.robot.commands.AutoCommands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.AutoConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCS extends CommandBase{

    private Drivetrain drivetrain;

    private double error;
    private Rotation2d heading;
    private double currentPitch, pitchRate;
    private double drivePower;
    private double balanceP, balanceFF, balanceAccelP;
    private double balanceTime;
    private boolean hasBalanced;

    public BalanceCS(){
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);
        balanceP = AutoConstants.kPCSBalanceDrive;
        balanceFF = -0.03;
        balanceAccelP = 0.008;

        SmartDashboard.putNumber("Balance CS error", error);
        SmartDashboard.putNumber("Balance CS drive power", drivePower);
        SmartDashboard.putNumber("Balance CS P value", balanceP);
        SmartDashboard.putNumber("Balance CS feedforward", balanceFF);
        SmartDashboard.putNumber("Balance CS Accel P value", balanceAccelP);

    }

    @Override

    public void execute(){
        heading = drivetrain.getHeadingAsRotation2d();
        currentPitch = drivetrain.getPitch();
        pitchRate = drivetrain.getPitchRate();

        error = AutoConstants.kCSGoalDegrees-currentPitch;
        SmartDashboard.putNumber("Balance CS error", error);

        balanceP = SmartDashboard.getNumber("Balance CS P value", 0.0);
        balanceFF = SmartDashboard.getNumber("Balance CS feedforward", 0.0);
        balanceAccelP = SmartDashboard.getNumber("Balance CS Accel P value", 0.0);

        drivePower = balanceFF*Math.signum(currentPitch) + Math.min(balanceP * error, 1);
        SmartDashboard.putNumber("Balance CS drive power", drivePower);

        //limit max power 
        if (Math.abs(drivePower) > 0.4) {
            drivePower = Math.copySign(0.4, drivePower);
        }

        if(Math.abs(pitchRate) > 0.5){
            drivePower -= balanceAccelP * pitchRate;
        }

        Translation2d chargeStationVector = new Translation2d(drivePower, heading);
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());

        if(!hasBalanced && isBalanced()){
            balanceTime = Timer.getFPGATimestamp();
            hasBalanced = true;
        }
        if(!isBalanced()){
            hasBalanced = false;
        }
    }

    //Called once the command ends or is interrupted
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
    }

    //Returns true when the command should end
    @Override
    public boolean isFinished(){
        double timeElapsedSinceBalance = Timer.getFPGATimestamp() - balanceTime;
        if(timeElapsedSinceBalance > 0.5 && isBalanced() && hasBalanced){
            return true; //Ends when we are in a specific interval
        }
        return false;
    }

    private boolean isBalanced(){
        return Math.abs(error) < AutoConstants.kCSAngleThresholdDegrees;
    }

}
