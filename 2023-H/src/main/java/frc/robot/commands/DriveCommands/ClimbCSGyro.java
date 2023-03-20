package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ClimbCSGyro extends CommandBase{
    private Drivetrain drivetrain;
    
    private BuiltInAccelerometer mRioAccel;
    private int state;
    private int debounceCount;
    private double robotSpeedSlow;
    private double robotSpeedFast;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;

    private double robotHeading;

    public ClimbCSGyro(double robotHeading){
        drivetrain = Drivetrain.getInstance();

        mRioAccel = new BuiltInAccelerometer();

        addRequirements(drivetrain);

        this.robotHeading = robotHeading;
    }

    @Override
    public void initialize() {
        state = 0;
        debounceCount = 0;

        /**********
         * CONFIG *
         **********/
        // Speed the robot drives while scoring/approaching station
        robotSpeedFast = 1.0;

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        robotSpeedSlow = 0.5;

        // Angle where the robot knows it is on the charge station
        onChargeStationDegree = 13.0;

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        levelDegree = 6.0;

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noise, but too high can make the auto run
        // slower
        debounceTime = 0.2;

    }

    @Override
    public void execute() {
        double robotSpeed = calculateRobotSpeed();

        Translation2d chargeStationVector = new Translation2d(robotSpeed, new Rotation2d(Math.toRadians(robotHeading)));
        drivetrain.drive(chargeStationVector, 0, true, new Translation2d());

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerveModules();
        drivetrain.lock();
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.getMatchTime() > 14.5){
            return true;
        }
        if(state == 4){
            return true;
        }
        return false;

    }

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0
    public double calculateRobotSpeed() {
        switch (state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                if (getTilt() > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 1:
                if (getTilt() < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            // on charge station, stop motors and wait for end of auto
            case 2:
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 3;
                    debounceCount = 0;
                    return 0;
                }
                if (getTilt() >= levelDegree) {
                    return 0.1;
                } else if (getTilt() <= -levelDegree) {
                    return -0.1;
                }
            case 3:
                return 0;
        }
        return 0;
    }

    public double getPitch() {
        return Math.atan2((-mRioAccel.getX()),
                Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
    }

    public double getRoll() {
        return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
    }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    public double getTilt() {
        double pitch = getPitch();
        double roll = getRoll();
        if ((pitch + roll) >= 0) {
            return Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return -Math.sqrt(pitch * pitch + roll * roll);
        }
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    
}