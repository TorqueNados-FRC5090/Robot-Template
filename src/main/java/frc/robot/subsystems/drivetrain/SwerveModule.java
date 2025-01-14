package frc.robot.subsystems.drivetrain;

// WPI imports
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Motor related imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;

// Math imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;

// Import constants
import static frc.robot.Constants.SwerveConstants.ModuleConstants.*;
import static frc.robot.Constants.SwerveConstants.MAX_TRANSLATION_SPEED;

/** This class represents a single swerve module */
public class SwerveModule extends SubsystemBase {
    private int moduleNumber;
    private SparkMax turnMotor;
    private SparkFlex driveMotor;
    private SwerveModuleState state;
    private SparkClosedLoopController driveController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;
    private PIDController turnController;
    private CANcoder angleEncoder;
    private double angleOffset;
    private double m_lastAngle;
    private Pose2d pose;

    /**
     * Constructs a SwerveModule.
     *
     * @param position The position of the module being constructed. (see enum)
     * @param driveMotor The ID of the driving motor.
     * @param turnMotor The ID of the turning motor.
     * @param absoluteEncoder The ID of the absolute encoder.
     * @param driveMotorInverted Whether the driving motor is inverted.
     * @param turningMotorInverted Whether the turning motor is inverted.
     * @param turningEncoderOffset The encoder's reading when pointing forward.
     */
    public SwerveModule(
        int moduleNumber,
        int driveMotorID,
        int turnMotorID,
        int absoluteEncoderID,
        boolean driveMotorInverted,
        boolean turningMotorInverted,
        double turningEncoderOffset) {

        // Initialize internal variables with values passed through paramsu
        this.moduleNumber = moduleNumber;
        angleOffset = turningEncoderOffset;

        // Construct and configure the driving motor
        driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
        SparkFlexConfig driveConfig = new SparkFlexConfig();

        driveConfig
            .smartCurrentLimit(40)
            .inverted(driveMotorInverted)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.6);
        driveConfig.encoder
            .positionConversionFactor(DRIVE_REVS_TO_M)
            .velocityConversionFactor(DRIVE_RPM_TO_MPS);
        driveConfig.closedLoop
            .pid(.2, 0, 0);
        driveConfig.signals
            .appliedOutputPeriodMs(100)
            .primaryEncoderVelocityPeriodMs(20)
            .primaryEncoderPositionPeriodMs(20)
            .warningsPeriodMs(500)
            .motorTemperaturePeriodMs(500)
            .busVoltagePeriodMs(500);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        // Construct and configure the turning motor
        turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();
        // The turning motor uses a WPILib PID controller, rather than a SPARK-included one
        turnController = new PIDController(.007, .00175, .0000625);
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        turnConfig
            .smartCurrentLimit(20)
            .inverted(turningMotorInverted)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.6);
        turnConfig.encoder
            .positionConversionFactor(TURNING_REVS_TO_DEG)
            .velocityConversionFactor(TURNING_REVS_TO_DEG / 60);
        turnConfig.signals
            .appliedOutputPeriodMs(100)
            .primaryEncoderVelocityPeriodMs(20)
            .primaryEncoderPositionPeriodMs(20)
            .warningsPeriodMs(500)
            .motorTemperaturePeriodMs(500)
            .busVoltagePeriodMs(500);

        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Construct and initialize the absolute encoder
        angleEncoder = new CANcoder(absoluteEncoderID);
        resetAngleToAbsolute();
    }

    /** Useful for iterating over modules like an array
     *  @return the number of this module */
    public int getModuleNumber() { return moduleNumber; }
    /** @return the direction this module is facing in degrees */
    public double getHeadingDegrees() { return turnEncoder.getPosition(); }
    /** @return the direction this module is facing as a {@link Rotation2d} object */
    public Rotation2d getHeadingRotation2d() { return Rotation2d.fromDegrees(getHeadingDegrees()); }
    /** @return How far this module has driven total in meters */
    public double getDriveMeters() { return driveEncoder.getPosition(); }
    /** @return The current speed of this module in m/sec */
    public double getDriveMetersPerSecond() { return driveEncoder.getVelocity(); }
    /** @return The current {@link SwerveModuleState state} of this module */
    public SwerveModuleState getState() { return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d()); }
    /** @return The {@link SwerveModulePosition position} of this module 
     *  expressed as the total distance driven and current heading */
    public SwerveModulePosition getPosition() { return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d()); }
    /** @return The current {@link Pose2d pose} of this module */
    public Pose2d getModulePose() { return pose; }
    /** Sets this module's {@link Pose2d pose} */
    public void setModulePose(Pose2d pose) { this.pose = pose; }
    

    /** Set the turning motor's encoder to absolute zero */
    public void resetAngleToAbsolute() {
        double angle = angleEncoder.getAbsolutePosition().getValueAsDouble() * 360 - angleOffset;
        turnEncoder.setPosition(angle);
    }

      /**Resets this modules drive encoder*/
      public void resetDriveEncoder(){
        driveEncoder.setPosition(0);
    }

    /**Resets this modules turn encoder */
    public void resetTurnEncoder(){
        turnEncoder.setPosition(0);
    }

    /**Resets both turn and drive encoders for this module */
    public void resetEncoders(){
        resetTurnEncoder();
        resetDriveEncoder();
    }

    /** Set the entire module to a desired {@link SwerveModuleState state}, controlling
     *  both the direction and speed at the same time
     * 
     *  @param desiredState The {@link SwerveModuleState state} to set the module to
     *  @param isOpenLoop True to control the driving motor via %power.
     *                    False to control the driving motor via velocity-based PID.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        state = optimize(desiredState, getHeadingRotation2d());

        if (isOpenLoop) {
            // Calculate the %power for the driving motor
            double percentOutput = state.speedMetersPerSecond / MAX_TRANSLATION_SPEED;
            // Send instruction to the motor
            driveMotor.set(percentOutput);
        } 
        else {
            // Set the driving motor's PID controller to the desired speed
            driveController.setReference(
                state.speedMetersPerSecond,
                SparkMax.ControlType.kVelocity,
                ClosedLoopSlot.kSlot1
            );
        }

        // Get the angle to turn the module to
        double angle =
            (Math.abs(state.speedMetersPerSecond) <= (MAX_TRANSLATION_SPEED * 0.01))
                ? m_lastAngle
                : state.angle.getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents Jittering.
    
        // Point turning motor at the target angle
        turnTo(angle);
    }

    /** Turn the module to point in some direction
     * 
     *  @param angle the target angle in degrees
     */
    public void turnTo(double angle) {
        double turnAngleError = Math.abs(angle - turnEncoder.getPosition());

        double pidOut = turnController.calculate(turnEncoder.getPosition(), angle);
        // if robot is not moving, stop the turn motor oscillating
        if (turnAngleError < .5 && Math.abs(state.speedMetersPerSecond) <= 0.03)
            pidOut = 0;

        turnMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());
    }

    public static SwerveModuleState optimize(
        SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle =
                placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;

        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

        while (newAngle < lowerBound)
            newAngle += 360;
        while (newAngle > upperBound)
            newAngle -= 360;

        if (newAngle - scopeReference > 180)
            newAngle -= 360;
        else if (newAngle - scopeReference < -180)
            newAngle += 360;

        return newAngle;
    }

    @Override // Called every 20ms
    public void periodic() {
        // Prints the position of the swerve module heading in degrees
        SmartDashboard.putNumber("Module " + moduleNumber + " Position", angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        // Prints the speed of the swerve module 
        SmartDashboard.putNumber("Module " + moduleNumber + " Speed", getDriveMetersPerSecond());
    }
}