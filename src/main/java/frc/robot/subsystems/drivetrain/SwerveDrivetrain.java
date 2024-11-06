package frc.robot.subsystems.drivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Math Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import java.util.Map;
import java.util.HashMap;

// Gyro imports
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Import constants
import frc.robot.Constants.SwerveConstants.ModulePosition;
import frc.robot.Constants.SwerveConstants;
import static frc.robot.Constants.SwerveIDs.*;
import static frc.robot.Constants.SwerveInversions.*;
import static frc.robot.Constants.SwerveModuleOffsets.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.DriveConstants.*;

/** This class represents the drivetrain on a 4 wheel swerve robot */
public class SwerveDrivetrain extends SubsystemBase {

    // Construct each swerve module
    /** The front left (FL) {@link SwerveModule}. Module number is 0 */
    private SwerveModule frontLeftModule = new SwerveModule(0,
        FL_DRIVE_ID,
        FL_TURN_ID,
        FL_ENCODER_ID,
        INVERT_FL_DRIVE,
        INVERT_FL_TURN,
        FL_OFFSET);

    /** The front right (FR) {@link SwerveModule}. Module number is 1 */
    private SwerveModule frontRightModule = new SwerveModule(1,
        FR_DRIVE_ID,
        FR_TURN_ID,
        FR_ENCODER_ID,
        INVERT_FR_DRIVE,
        INVERT_FR_TURN,
        FR_OFFSET);

    /** The rear left (RL) {@link SwerveModule}. Module number is 2 */
    private SwerveModule rearLeftModule = new SwerveModule(2,
        RL_DRIVE_ID,
        RL_TURN_ID,
        RL_ENCODER_ID,
        INVERT_RL_DRIVE,
        INVERT_RL_TURN,
        RL_OFFSET);

    /** The rear right (RR) {@link SwerveModule}. Module number is 3 */
    private SwerveModule rearRightModule = new SwerveModule(3,
        RR_DRIVE_ID,
        RR_TURN_ID,
        RR_ENCODER_ID,
        INVERT_RR_DRIVE,
        INVERT_RR_TURN,
        RR_OFFSET);

    /** A {@link HashMap} associating each {@link SwerveModule module} with its {@link ModulePosition position} */
    private final HashMap<ModulePosition, SwerveModule> swerveModules =
        new HashMap<>(
            Map.of(
                ModulePosition.FRONT_LEFT,
                frontLeftModule,

                ModulePosition.FRONT_RIGHT,
                frontRightModule,

                ModulePosition.REAR_LEFT,
                rearLeftModule,

                ModulePosition.REAR_RIGHT,
                rearRightModule));


    // Declare and initialize the limiters used to slew instructions
    private final SlewRateLimiter slewX = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewY = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewRot = new SlewRateLimiter(ROTATION_SLEW);

    /** The gyro is used to help keep track of where the robot is facing */
    private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    /** While the robot is in field centric mode, forward is a defined direction.
     *  Conversely, if the robot is not in field centric mode, it is robot centric.
     *  While the robot is in robot centric mode, forward is whichever direction the robot is facing. */
    private boolean isFieldCentric = true;

    /** Used to track the robot's position as it moves */
    private SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(
            SwerveConstants.SWERVE_KINEMATICS,
            getHeadingRotation2d(),
            getModulePositions(),
            new Pose2d());
    
    /** Constructs a drivetrain {@link SubsystemBase subsystem} */
    public SwerveDrivetrain() {
        gyro.reset();
    }

    /** Preprocess a drive instruction for driver controlled side-to-side movement
     *  @param input The X input coming from a driver's joystick
     *  @return The processed output ready to be sent to the drivetrain */
    public double preprocessX(double input) {
        double deadbanded = MathUtil.applyDeadband(input, .05);
        double squared = -Math.signum(deadbanded) * Math.pow(deadbanded, 2);
        double output = slewX.calculate(squared);

        return output;
    }
    /** Preprocess a drive instruction for driver controlled forward/back movement
     *  @param input The Y input coming from a driver's joystick
     *  @return The processed output ready to be sent to the drivetrain */
    public double preprocessY(double input) {
        double deadbanded = MathUtil.applyDeadband(input, .05);
        double squared = -Math.signum(deadbanded) * Math.pow(deadbanded, 2);
        double output = slewY.calculate(squared);

        return output;
    }
    /** Preprocess a drive instruction for driver controlled rotation
     *  @param input The rotation input coming from a driver's joystick
     *  @return The processed output ready to be sent to the drivetrain */
    public double preprocessRot(double input) {
        double deadbanded = MathUtil.applyDeadband(input, .1);
        double squared = -Math.signum(deadbanded) * Math.pow(deadbanded, 2);
        double output = slewRot.calculate(squared);

        return output;
    }

    /** Drives the robot. This is best used for driving with joysticks.
     *  For programmatic drivetrain control, consider using sendDrive instead. 
     * 
     * @param inputX The left/right translation instruction
     * @param inputY The forward/back translation instruction
     * @param inputRot The rotational instruction
    */
    public void drive(double inputX, double inputY, double inputRot) {
        double outX = preprocessX(inputX);
        double outY = preprocessY(inputY);
        double outRot = preprocessRot(inputRot);

        // Send the processed output to the drivetrain
        sendDrive(outX, outY, outRot, true);
    }

    /** Sends driving instructions to the motors that drive the robot.
     *  
     * @param translationX The left/right translation instruction
     * @param translationY The forward/back translation instruction
     * @param rotation The rotational instruction
     * @param isOpenLoop True to control the driving motor via %power.
     *                   False to control the driving motor via velocity-based PID.
    */
    public void sendDrive( double translationX, double translationY, double rotation, boolean isOpenLoop ) {
        // Convert inputs from % to m/sec
        translationY *= MAX_TRANSLATION_SPEED;
        translationX *= MAX_TRANSLATION_SPEED;
        rotation *= MAX_ROTATION_SPEED;

        ChassisSpeeds chassisSpeeds =
            isFieldCentric
                // Calculate field relative instructions if isFieldCentric is true
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translationY, translationX, rotation, getHeadingRotation2d())
                // Calculate robot centric instructions if isFieldCentric is false
                : new ChassisSpeeds(translationY, translationX, rotation);

        // Convert ChassisSpeed instructions to useable SwerveModuleStates
        SwerveModuleState[] moduleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // Normalize output if any of the modules would be instructed to go faster than possible
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_TRANSLATION_SPEED);

        // Send instructions to each module
        for (SwerveModule module : swerveModules.values())
            module.setDesiredState(moduleStates[module.getModuleNumber()], isOpenLoop);
    }

    /** Used by pathplanner to control the robot in robotspace */
    public void driveRobotRelative(ChassisSpeeds speeds) { 
        // Convert ChassisSpeed instructions to useable SwerveModuleStates
        SwerveModuleState[] moduleStates = SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        // Normalize output if any of the modules would be instructed to go faster than possible
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_TRANSLATION_SPEED);

        // Send instructions to each module
        for (SwerveModule module : swerveModules.values())
            module.setDesiredState(moduleStates[module.getModuleNumber()], true);
    }

    // Misc getters
    /** @return The current direction the robot is facing in degrees */
    public double getHeadingDegrees() { return -Math.IEEEremainder(gyro.getAngle(), 360); }
    /** @return The current direction the robot is facing as a {@link Rotation2d} object */
    public Rotation2d getHeadingRotation2d() { return Rotation2d.fromDegrees(getHeadingDegrees()); }
    /** Reset the heading of the robot, effectively changing the orientation of the field */
    public void resetHeading() { gyro.reset(); }
    /** @return The position in meters and direction of the robot in degrees as a {@link Pose2d} object */
    public Pose2d getPoseMeters() { return odometry.getPoseMeters(); }
    /** @param moduleNumber The index of the module 
     *  @return The {@link SwerveModule swerve module} at that index */
    public SwerveModule getSwerveModule(int moduleNumber) { return swerveModules.get(ModulePosition.values()[moduleNumber]); }
    /** @param position The {@link ModulePosition position} of the module
     *  @return The {@link SwerveModule swerve module} at that position */
    public SwerveModule getSwerveModule(ModulePosition position) { return swerveModules.get(ModulePosition.FRONT_LEFT); }
    /** @return A ChassisSpeeds object describing the motion of the robot */
    public ChassisSpeeds getChassisSpeeds() { return SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()); }
    
    // Methods related to field orientation
    /** @return Whether or not the robot is in field oriented mode */
    public boolean isFieldCentric() { return isFieldCentric; }
    /** @return The opposite of isFieldCentric() */
    public boolean isRobotCentric() { return !isFieldCentric; }
    /** @param isFieldCentric Whether the robot should be set to field centric or not */
    public void setFieldCentric(boolean isFieldCentric) { this.isFieldCentric = isFieldCentric; }
    /** @param isFieldCentric Whether the robot should be set to robot centric or not */
    public void setRobotCentric(boolean isRobotCentric) { this.isFieldCentric = !isRobotCentric; }
    /** Sets the robot to field centric if currently robot centric and vice versa */
    public void toggleFieldCentric() { this.isFieldCentric = !this.isFieldCentric; }

    /** Resets the wheels of the robot to point forward */
    public void zeroWheels() { 
         for (SwerveModule module : swerveModules.values())
            module.setDesiredState(
                new SwerveModuleState(0, new Rotation2d(0)),    
                true);
    }

    /** @return An array containing the current {@link SwerveModuleState state} of each module */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
            swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
            swerveModules.get(ModulePosition.REAR_LEFT).getState(),
            swerveModules.get(ModulePosition.REAR_RIGHT).getState()
        };
    }

    /** Set the state of each module at once
     *  @param states An array containing the desired {@link SwerveModuleState state} of each module */
    public void setModuleStates(SwerveModuleState[] states) {
        // Normalize output if any of the modules would be instructed to go faster than possible
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_TRANSLATION_SPEED);

        // Send instructions to each module
        for (SwerveModule module : swerveModules.values())
            module.setDesiredState(states[module.getModuleNumber()], true);
    }

    /** Set the state of each to 0,0 */
    public void resetModuleStates() {
        // Send instructions to each module
        for (SwerveModule module : swerveModules.values())
            module.setDesiredState(new SwerveModuleState(), true);
    }

    /** @return An array containing the current {@link SwerveModulePosition position} of each module */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
            swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
            swerveModules.get(ModulePosition.REAR_LEFT).getPosition(),
            swerveModules.get(ModulePosition.REAR_RIGHT).getPosition()
        };
    }

    /** Updates the odometry of the robot using the {@link SwerveModulePosition position} 
     *  of each module and the current heading of the robot */
    public void updateOdometry() {
        odometry.update(getHeadingRotation2d(), getModulePositions());

        for (SwerveModule module : swerveModules.values()) {
        var modulePositionFromChassis =
            MODULE_TRANSLATIONS[module.getModuleNumber()]
                .rotateBy(getHeadingRotation2d())
                .plus(getPoseMeters().getTranslation());
        module.setModulePose(
            new Pose2d(
                modulePositionFromChassis,
                module.getHeadingRotation2d().plus(getHeadingRotation2d())));
        }
    }

    /** Sets the odometry of the robot using a given pose
     *  @param pose The pose of the robot */
    public void setOdometry( Pose2d pose) {
        odometry.resetPosition(
            getHeadingRotation2d(),
            getModulePositions(),
            pose);
    }

    /** Resets the odometry of the robot */
    public void resetOdometry() {
        odometry.resetPosition(
            getHeadingRotation2d(),
            getModulePositions(),
            new Pose2d());
    }

    @Override // Called every 20ms
    public void periodic() {
        updateOdometry();
        // Prints the current heading in degrees
        SmartDashboard.putNumber("Heading in Degrees", getHeadingDegrees());
        // Prints if the robot is in field or robot
        SmartDashboard.putBoolean("Field Centric", isFieldCentric());
    }
}
