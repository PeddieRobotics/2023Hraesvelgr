package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.AutoConstants;

public class ClimbCSGyro extends CommandBase{
    private Drivetrain drivetrain;
    
    private int state;
    private int debounceCount;

    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;

    private double fieldHeading, gyroHeading, initialPitch, currentPitch, approachSpeed, climbSpeed;

    // Full constructor with all 6 parameters for the climb charge station algorithm.
    public ClimbCSGyro(double fieldHeading, double approachSpeed, double climbSpeed, double debounceTime, double onChargeStationDegree, double levelDegree){
        drivetrain = Drivetrain.getInstance();

        addRequirements(drivetrain);

        this.fieldHeading = fieldHeading;
        this.approachSpeed = approachSpeed;
        this.climbSpeed = climbSpeed;
        this.debounceTime = debounceTime;
        this.onChargeStationDegree = onChargeStationDegree;
        this.levelDegree = levelDegree;

        /**********
         * CONFIG *
         **********/
        // Speed the robot drives while approaching station
        SmartDashboard.putNumber("CLIMB: Approach speed", approachSpeed);

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        SmartDashboard.putNumber("CLIMB: Climb speed", climbSpeed);

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noise, but too high can make the auto run
        // slower
        SmartDashboard.putNumber("CLIMB: Debounce time", debounceTime);

        // Angle where the robot knows it is on the charge station
        SmartDashboard.putNumber("CLIMB: On station angle", onChargeStationDegree);

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence.
        SmartDashboard.putNumber("CLIMB: Stop angle", levelDegree);
    }

    // Typical constructor. Used if the default parameters are fine and don't need to be overwritten.
    public ClimbCSGyro(double fieldHeading, double approachSpeed, double climbSpeed){
        this(fieldHeading, approachSpeed, climbSpeed, AutoConstants.kCSDebounceTime, AutoConstants.kOnCSDegree, AutoConstants.kCSLevelDegree);
    }

    @Override
    public void initialize() {
        state = 0;
        debounceCount = 0;

        // Check if gyro's zero is the same as field zero.
        if(drivetrain.getFlipped()){
            gyroHeading = 180-fieldHeading;
        }

        initialPitch = drivetrain.getPitch();

        /*
         * More config. In initialize method to allow for live dashboard tuning.
         **/

        // Speed the robot drives while approaching station
        approachSpeed = SmartDashboard.getNumber("CLIMB: Approach speed", approachSpeed);

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        climbSpeed = SmartDashboard.getNumber("CLIMB: Climb speed", climbSpeed);
        
        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noise, but too high can make the auto run
        // slower
        debounceTime = SmartDashboard.getNumber("CLIMB: Debounce time", 0.05);

        // Angle where the robot knows it is on the charge station
        onChargeStationDegree = SmartDashboard.getNumber("CLIMB: On station angle", 13.0);

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence.
        levelDegree = SmartDashboard.getNumber("CLIMB: Stop angle", 10.0);

    }

    @Override
    public void execute() {
        double robotSpeed = calculateRobotSpeed();
        currentPitch = drivetrain.getPitch()-initialPitch;
        SmartDashboard.putNumber("CLIMB: Current pitch angle", currentPitch);
        SmartDashboard.putNumber("CLIMB: Current speed", robotSpeed);
        SmartDashboard.putNumber("CLIMB: Current state", state);

        Translation2d chargeStationVector = new Translation2d(robotSpeed, new Rotation2d(Math.toRadians(gyroHeading)));
        if(state != 2){
            drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
        }
        else{
            drivetrain.lock();
            Blinkin.getInstance().lockedWheels();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.lock();
        Blinkin.getInstance().lockedWheels();
    }

    @Override
    public boolean isFinished() {
        if(state == 2){
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
                if(Math.abs(currentPitch) > onChargeStationDegree) {
                    debounceCount++;
                }
                if(debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    return climbSpeed;
                }
                return approachSpeed;
            // driving up charge station, drive slower, stopping when level
            case 1:
                if(currentPitch > 0){
                    if (currentPitch < levelDegree) {
                        debounceCount++;
                    }
                }
                else{
                    if (currentPitch > -levelDegree) {
                        debounceCount++;
                    }
                }

                if(debounceCount > secondsToTicks(debounceTime)) {
                    state = 2;
                    debounceCount = 0;
                    return 0;
                }
                return climbSpeed;
            // on charge station, stop motors and wait for end of auto
            case 2:
                return 0;
        }
        return 0;
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    
}