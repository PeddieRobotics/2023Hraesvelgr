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

    private double onChargeStationDegree;

    private double fieldHeading, gyroHeading, currentPitch, approachSpeed, climbSpeed;

    private double initialXPos, initialClimbXPos, currentDistance, currentClimbDistance;

    private double overrunMaxApproachDist, overrunRemainingClimbDist;

    private double initialDrivebackXPos, currentDrivebackDistance;

    private double robotSpeed;

    private Blinkin blinkin;

    // Full constructor with all 6 parameters for the climb charge station algorithm.
    public ClimbCSGyro(double fieldHeading, double approachSpeed, double climbSpeed, double onChargeStationDegree, double overrunMaxApproachDist){
        drivetrain = Drivetrain.getInstance();

        addRequirements(drivetrain);

        this.fieldHeading = fieldHeading;
        this.approachSpeed = approachSpeed;
        this.climbSpeed = climbSpeed;
        this.onChargeStationDegree = onChargeStationDegree;
        this.overrunMaxApproachDist = overrunMaxApproachDist;

        blinkin = Blinkin.getInstance();

        /**********
         * CONFIG *
         **********/
        // Speed the robot drives while approaching station
        SmartDashboard.putNumber("CLIMB: Approach speed", approachSpeed);

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        SmartDashboard.putNumber("CLIMB: Climb speed", climbSpeed);

        // Angle where the robot knows it is on the charge station
        SmartDashboard.putNumber("CLIMB: On station angle", onChargeStationDegree);

        SmartDashboard.putNumber("CLIMB: Overrun max approach dist", overrunMaxApproachDist);

        SmartDashboard.putNumber("DATA: Current pitch angle", currentPitch);
        SmartDashboard.putNumber("DATA: Current speed", robotSpeed);
        SmartDashboard.putNumber("DATA: Current state", state);
        SmartDashboard.putNumber("DATA: Initial x pos", initialXPos);
        SmartDashboard.putNumber("DATA: Initial climb x pos", initialClimbXPos);
        SmartDashboard.putNumber("DATA: Initial driveback x pos", initialDrivebackXPos);
        SmartDashboard.putNumber("DATA: Current total distance", currentDistance);
        SmartDashboard.putNumber("DATA: Current climb distance", currentClimbDistance);
        SmartDashboard.putNumber("DATA: Current driveback x pos", currentDrivebackDistance);

    }

    // Typical constructor. Used if the default parameters are fine and don't need to be overwritten.
    public ClimbCSGyro(double fieldHeading, double approachSpeed, double climbSpeed){
        this(fieldHeading, approachSpeed, climbSpeed, AutoConstants.kOnCSDegree, AutoConstants.kCSOverrunMaxApproachDist);
    }

    @Override
    public void initialize() {
        state = 0;

        initialXPos = drivetrain.getOdometry().getEstimatedPosition().getX();
        initialClimbXPos = 0.0;
        currentDistance = 0.0;
        currentClimbDistance = 0.0;
        robotSpeed = 0;
        initialDrivebackXPos = 0.0;
        currentDrivebackDistance = 0.0;

        // Check if gyro's zero is the same as field zero.
        if(drivetrain.getFlipped()){
            gyroHeading = 180-fieldHeading;
        }

        // initialPitch = drivetrain.getPitch();

        /*
         * More config. In initialize method to allow for live dashboard tuning.
         **/

        // Speed the robot drives while approaching station
        approachSpeed = SmartDashboard.getNumber("CLIMB: Approach speed", approachSpeed);

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        climbSpeed = SmartDashboard.getNumber("CLIMB: Climb speed", climbSpeed);
        
        // Angle where the robot knows it is on the charge station
        onChargeStationDegree = SmartDashboard.getNumber("CLIMB: On station angle", AutoConstants.kOnCSDegree);

        overrunMaxApproachDist = SmartDashboard.getNumber("CLIMB: Overrun max approach dist", AutoConstants.kCSOverrunMaxApproachDist);
        
        overrunRemainingClimbDist = SmartDashboard.getNumber("CLIMB: Overrun remaining climb dist", AutoConstants.kCSOverrunRemainingClimbDist);        

    }

    @Override
    public void execute() {
        robotSpeed = calculateRobotSpeed();
        currentPitch = Math.abs(drivetrain.getPitchAverage());

        currentDistance = Math.abs(drivetrain.getOdometry().getEstimatedPosition().getX() - initialXPos);

        SmartDashboard.putNumber("DATA: Current pitch angle", currentPitch);
        SmartDashboard.putNumber("DATA: Current speed", robotSpeed);
        SmartDashboard.putNumber("DATA: Current state", state);
        SmartDashboard.putNumber("DATA: Initial x pos", initialXPos);
        SmartDashboard.putNumber("DATA: Initial climb x pos", initialClimbXPos);
        SmartDashboard.putNumber("DATA: Current total distance", currentDistance);
        SmartDashboard.putNumber("DATA: Current climb distance", currentClimbDistance);

        Translation2d chargeStationVector;
        if(state == 0 || state == 1){
            chargeStationVector = new Translation2d(robotSpeed, new Rotation2d(Math.toRadians(gyroHeading)));
            drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
        }
        else if(state == 2 || state == 3){
            chargeStationVector = new Translation2d(robotSpeed, new Rotation2d(Math.toRadians(180-gyroHeading)));
            drivetrain.drive(chargeStationVector, 0, true, new Translation2d());
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if(state == 3){
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
                // Overran detection window
                // Set climb distance to a shorter, more appropriate distance.
                if(currentDistance > overrunMaxApproachDist){
                    blinkin.gyroClimbOverrun();
                    state = 2;
                    return 0;
                }
                // Successful gyro detection of charge station
                else if(currentPitch > onChargeStationDegree){
                    blinkin.specialOperatorFunctionality();
                    initialClimbXPos = drivetrain.getOdometry().getEstimatedPosition().getX();
                    state = 1;
                }

                return approachSpeed;

            // driving up charge station, drive slower, stopping when level
            case 1:
                currentClimbDistance = Math.abs(drivetrain.getOdometry().getEstimatedPosition().getX() - initialClimbXPos);
                
                // Overran detection window
                if(currentDistance > overrunMaxApproachDist){
                    blinkin.gyroClimbOverrun();
                    state = 2;
                    initialDrivebackXPos = drivetrain.getOdometry().getEstimatedPosition().getX();
                    return 0;
                }

                if(currentPitch < 11 && currentClimbDistance > 0.5) {
                    blinkin.gyroClimbSuccess();
                    state = 2;
                    initialDrivebackXPos = drivetrain.getOdometry().getEstimatedPosition().getX();
                    return 0;
                }

                return 0.75;

            // reacted to angle drop but it's slightly too slow; so reverse a small fixed amount amd stp[]
            case 2:
                currentDrivebackDistance = Math.abs(drivetrain.getOdometry().getEstimatedPosition().getX() - initialDrivebackXPos);
                blinkin.lockedWheels();
                if(currentDrivebackDistance > 0.1){
                    state = 3;
                    return 0;
                }
                return 0.75;

            case 3:
                return 0;
        }
        return 0;
    }
    
}