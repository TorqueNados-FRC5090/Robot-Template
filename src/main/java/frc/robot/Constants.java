package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.TunerConstants;

/** Every number you might want to tune lives here, grouped by what it belongs to. */
public final class Constants {

    /** CAN bus names. "Jarvis" is the CANivore the drivetrain is on (set in TunerConstants). */
    public static final String kDriveCANBus = "Jarvis";

/* -------------- CONTROLLER CONSTANTS -------------- */

    public static final class OperatorConstants {
        /** USB port of the driver's Xbox controller in the Driver Station */
        public static final int kDriverPort = 0;
        /** Stick inputs smaller than this (as a fraction of full stick) are ignored */
        public static final double kDeadband = 0.1;
        /** Speed multiplier while the right trigger is held (slow mode) */
        public static final double kSlowModeMultiplier = 0.4;
        /** Battery voltage where the controller starts to rumble. Full rumble at the brownout voltage. */
        public static final double kRumbleStartVolts = 7.5;
        /** How fast the rumble fades after a dip (per 20ms loop). Dips last milliseconds, so this lets the driver feel them. */
        public static final double kRumbleDecayPerLoop = 0.03;
    }

/* -------------- DRIVETRAIN CONSTANTS -------------- */

    public static final class DriveConstants {
        /** Top driving speed. Comes from TunerConstants so there is only one number to keep correct. */
        public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        /** Top spinning speed. Raise it if the drivers want faster turning. */
        public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

/* -------------- PATHPLANNER CONSTANTS -------------- */

    public static final class AutoConstants {
        /** How hard PathPlanner corrects position error (meters -> m/s) */
        public static final PIDConstants kTranslationPID = new PIDConstants(5.0, 0.0, 0.0);
        /** How hard PathPlanner corrects heading error (radians -> rad/s) */
        public static final PIDConstants kRotationPID = new PIDConstants(5.0, 0.0, 0.0);

        /** Robot mass with bumpers and battery, in kg. Keep in sync with the PathPlanner app's Robot Config. */
        private static final double kRobotMassKg = 74.088;
        /** Robot moment of inertia, in kg*m^2 */
        private static final double kRobotMOI = 6.883;
        /** Wheel coefficient of friction (1.2 is typical for new tread) */
        private static final double kWheelCOF = 1.2;
        /** Drive motor current limit, in amps. Matches kSlipCurrent in TunerConstants. */
        private static final double kDriveCurrentLimit = 60;

        /**
         * Backup PathPlanner robot config built from TunerConstants. AutonContainer normally loads the config
         * saved by the PathPlanner app (deploy/pathplanner/settings.json); this is only used if that fails.
         */
        public static final RobotConfig kFallbackConfig = new RobotConfig(
            kRobotMassKg, kRobotMOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                DriveConstants.kMaxSpeed,
                kWheelCOF,
                DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                kDriveCurrentLimit,
                1),
            moduleLocation(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            moduleLocation(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            moduleLocation(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            moduleLocation(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));

        private static Translation2d moduleLocation(double xMeters, double yMeters) {
            return new Translation2d(xMeters, yMeters);
        }
    }

/* -------------- VISION CONSTANTS -------------- */

    public static final class VisionConstants {
        /**
         * Names of every Limelight on the robot, exactly as set in each Limelight's web page (http://limelight.local:5801).
         * Add or remove names here to add or remove cameras. Each one feeds the pose estimator.
         */
        public static final String[] kCameraNames = { "limelight" };

        /** Ignore vision while spinning faster than this (rotations per second); the images blur and lag. */
        public static final double kMaxSpinRps = 2.0;

        /** How much to trust MegaTag2 X/Y, in meters (bigger = trust less). Heading always comes from the Pigeon. */
        public static final double kMegaTag2XYStdDev = 0.7;
        /** How much to trust MegaTag1 while disabled, in meters/radians. Used to fix the starting heading. */
        public static final double kMegaTag1StdDev = 0.5;
        /** MegaTag1 needs at least this many tags before it is allowed to fix the heading */
        public static final int kMegaTag1MinTags = 2;
    }
}
