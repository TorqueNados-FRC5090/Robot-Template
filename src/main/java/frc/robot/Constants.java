package frc.robot;
// Imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/* 
 * To access numbers in this file, import or statically import one of its subclasses:
 * example:
 * import static frc.robot.Constants.ControllerPorts.*;
 * import frc.robot.Constants.DriveConstants;
 */
public final class Constants {

    /* -------------- IDs -------------- */

    /** Ports used by controllers. */
    public static final class ControllerPorts {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    /** IDs used by the drivetrain.
        3X for turning, 4X for driving, 5X for abs encoders. */
        public static final class WestCoastIDs {
            // Left Motor IDs
            public static final int LEFT1_ID = 2;
            public static final int LEFT2_ID = 3;
            // Right Motor IDs
            public static final int RIGHT1_ID = 6;
            public static final int RIGHT2_ID = 7;
        }

    /** IDs used by the swerve drivetrain.
        3X for turning, 4X for driving, 5X for abs encoders. */
    public static final class SwerveIDs {
        // Front left module
        public static final int FL_DRIVE_ID = 40;
        public static final int FL_TURN_ID = 30;
        public static final int FL_ENCODER_ID = 50;
        // Front right module
        public static final int FR_DRIVE_ID = 41;
        public static final int FR_TURN_ID = 31;
        public static final int FR_ENCODER_ID = 51;
        // Rear left module
        public static final int RL_DRIVE_ID = 42;
        public static final int RL_TURN_ID = 32;
        public static final int RL_ENCODER_ID = 52;
        // Rear right module
        public static final int RR_DRIVE_ID = 43;
        public static final int RR_TURN_ID = 33;
        public static final int RR_ENCODER_ID = 53;
    }

    /* -------------- SUBSYTEM CONSTANTS -------------- */

    /** Turning a module to absolute 0 minus its offset will point it forward */
    public static final class SwerveModuleOffsets {
        public static final double FL_OFFSET = 0;
        public static final double FR_OFFSET = 0;
        public static final double RL_OFFSET = 0;
        public static final double RR_OFFSET = 0;
    }

    /** Whether or not each swerve component should be inverted/reversed */
    public static final class SwerveInversions {
        // Whether each driving motor should be inverted
        public static final boolean INVERT_FL_DRIVE = false;
        public static final boolean INVERT_RL_DRIVE = false;
        public static final boolean INVERT_FR_DRIVE = true;
        public static final boolean INVERT_RR_DRIVE = true;

        // Whether each turning motor should be inverted
        public static final boolean INVERT_FL_TURN = true;
        public static final boolean INVERT_FR_TURN = true;
        public static final boolean INVERT_RL_TURN = true;
        public static final boolean INVERT_RR_TURN = true;
    }

    /** Constants related to swerve calculations */
    public static final class SwerveConstants {
        /** The distance between the left and right wheels */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22);
        /** The distance between the front and rear wheels */
        public static final double WHEEL_BASE = Units.inchesToMeters(27);

        /** An array containing the position of each module as a {@link Translation2d} object */
        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        };

        // Kinematics are used to calculate how each module needs to move
        // in order to move the robot as a whole in a certain way

        /** Standard kinematics with center of rotation located at the center of the robot */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        /** The max speed the robot is allowed to drive in m/sec */
        public static final double MAX_TRANSLATION_SPEED = 4.5;
        /** The max speed the robot is allowed to spin in rads/sec */
        public static final double MAX_ROTATION_SPEED = Math.PI;
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_ROTATION_SPEED, Math.PI * 2);

        public static final class ModuleConstants {
            /** The ratio of the drive motors on the workhorse chassis */
            public static final double DRIVE_RATIO_SLOW = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));
            /** The ratio of the drive motors on the robot */
            public static final double DRIVE_RATIO_FAST = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
            /** The ratio of the turning motors */
            public static final double TURN_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
            
            /** The diameter of the wheels measured in meters */
            public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.94);

            /** Drive motor revolutions * DRIVE_REVS_TO_M = distance in meters */
            public static final double DRIVE_REVS_TO_M = ((WHEEL_DIAMETER * Math.PI) / DRIVE_RATIO_FAST);

            // 1 RPM * DRIVE_REVS_TO_M = speed in m/min. Divide by 60 to find m/sec
            /** Drive motor RPM * DRIVE_RPM_TO_MPS = speed in m/sec */
            public static final double DRIVE_RPM_TO_MPS = DRIVE_REVS_TO_M / 60.0;

            /** Turning motor revolutions * TURNING_REVS_TO_DEG = Turning motor total degrees turned */
            public static final double TURNING_REVS_TO_DEG =  360.0 / TURN_RATIO;
        }
    
        /** Enum representing the four possible positions a module can occupy */
        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }
    }

  public static final class DriveConstants {
            /** Higher values make the robot drive more aggressively */
            public static final double TRANSLATION_SLEW = 4;
            /** Higher values make the robot spin more aggressively */
            public static final double ROTATION_SLEW = 6;
    
            /** Translation instructions closer to 0 than the deadband will be set to 0 */
            public static final double TRANSLATION_DEADBAND = .05;
            /** Rotation instructions closer to 0 than the deadband will be set to 0 */
            public static final double ROTATION_DEADBAND = .1;
    }
}
