package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor, angleMotor;

    private SwerveModuleState state;

    private final RelativeEncoder m_driveEncoder;
    private final AnalogPotentiometer m_turningEncoder;
    private final AbsoluteEncoder m_absoluteTurningEncoder;

    private final PIDController anglePIDController, drivePIDController;

    private final SimpleMotorFeedforward driveFeedForward, angleFeedForward;

    private double finalAngle;

    private int driveMotorCanId, angleMotorCanId;

    public SwerveModule(int driveMotorCanId, int angleMotorCanId, double driveP, double angleP,
            int turningEncoderChannel, double angularOffset) {

        driveMotor = new CANSparkMax(driveMotorCanId, MotorType.kBrushless);

        driveMotor.setSmartCurrentLimit(DriveConstants.kTranslation);
        driveMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);

        driveMotor.setInverted(false);

        m_driveEncoder = driveMotor.getEncoder();
        m_driveEncoder.setVelocityConversionFactor(1 / (Constants.DRIVE_GEAR_RATIO * Constants.METERS_PER_SEC_TO_RPM));
        driveMotor.setIdleMode(IdleMode.kBrake);

        drivePIDController = new PIDController(driveP, 0.0, 0.0);
        driveFeedForward = new SimpleMotorFeedforward(0.015, 0.2850);

        this.driveMotorCanId = driveMotorCanId;

        driveMotor.burnFlash();

        angleMotor = new CANSparkMax(angleMotorCanId, MotorType.kBrushless);

        angleMotor.setSmartCurrentLimit(DriveConstants.kRotation);
        angleMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);

        angleMotor.setInverted(true);

        m_turningEncoder = new AnalogPotentiometer(turningEncoderChannel, 2.0 * Math.PI, angularOffset);

        angleMotor.setIdleMode(IdleMode.kBrake);

        this.angleMotorCanId = angleMotorCanId;

        angleMotor.burnFlash();

        SmartDashboard.putNumber(angleMotorCanId + "anglep", 0.15);
        // anglePIDController = new PIDController(angleP, 0, 0);
        anglePIDController = new PIDController(0.20, 0, 0);
        anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
        // angleFeedForward = new SimpleMotorFeedforward(0.00015, 0.00015);
        angleFeedForward = new SimpleMotorFeedforward(0.00015, 0.00015);

        // angleMotor.getPIDController().setP(0.15);
        // driveMotor.getPIDController().setP(0.15);

    }

    public void setDriveMotor(double setpoint) {
        driveMotor.set(setpoint);
    }

    public void setAngleMotor(double setpoint) {
        angleMotor.set(setpoint);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoder()));
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(),getCurrentState().angle);
    }

    public SwerveModuleState getDesiredState() {
        return state;
    }

    public void setDesiredState(SwerveModuleState desiredState) {// gives the module its desired state and moves it to
                                                                 // its desired wheel velocity and angular position
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoder()));

        final double driveOutput = drivePIDController.calculate(m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);
        final double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);

        final double finalDriveOutput = driveOutput + driveFF;
        driveMotor.set(finalDriveOutput);

        final double angleOutput = anglePIDController.calculate(getTurnEncoder(), state.angle.getRadians());
        final double angleFF = angleFeedForward.calculate(state.angle.getDegrees());

        final double finalAngleOutput = angleOutput + angleFF;
        angleMotor.set(finalAngleOutput);
    }

    public void resetSwerveModule(){

        SwerveModuleState resetState = new SwerveModuleState(0, new Rotation2d(90));
        driveMotor.set(0);

        final double angleOutput = anglePIDController.calculate(getTurnEncoder(), resetState.angle.getRadians());
        final double angleFF = angleFeedForward.calculate(resetState.angle.getDegrees());

        final double finalAngleOutput = angleOutput + angleFF;
        angleMotor.set(finalAngleOutput);
    }

    public void setPIDFDrive(double p, double i, double d, double ff) {
        driveMotor.getPIDController().setP(p);
        driveMotor.getPIDController().setI(i);
        driveMotor.getPIDController().setD(d);
        driveMotor.getPIDController().setFF(ff);

    }

    public void setPIDFAngle(double p, double i, double d, double ff) {
        angleMotor.getPIDController().setP(p);
        angleMotor.getPIDController().setI(i);
        angleMotor.getPIDController().setD(d);
        angleMotor.getPIDController().setFF(ff);

    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public double getTurnEncoder() {
        return m_turningEncoder.get();
    }

    public void putShuffleboard() {

        SmartDashboard.putNumber(driveMotorCanId + " Wheel RPM", m_driveEncoder.getVelocity());
        SmartDashboard.putNumber(angleMotorCanId + " Angle Motor Angle", m_turningEncoder.get());
    }


    public void getFromShuffleboard(){
        double anglep = SmartDashboard.getNumber(angleMotorCanId + "anglep", 0.15);
        anglePIDController.setP(anglep);
    }

}  
