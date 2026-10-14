package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer {
    private final RobotContainer robotContainer;
    private final CommandSwerveDrivetrain drivetrain;

    /** Constructs an AutonContainer object */
    public AutonContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        // Named commands must be registered BEFORE any auto is built, or PathPlanner can't find them
        registerNamedCommands();

        // Load the robot config saved by the PathPlanner app (Settings -> Robot Config).
        // Fall back onto the config in Constants because it's better than crashing.
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner settings.json failed to load, using fallback config: " + e.getMessage(), e.getStackTrace());
            config = AutoConstants.kFallbackConfig;
        }

        AutoBuilder.configure(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::getChassisSpeeds,
            drivetrain::driveRobotRelative,
            new PPHolonomicDriveController(AutoConstants.kTranslationPID, AutoConstants.kRotationPID),
            config,
            // Paths are drawn on the blue side; flip them when we're red
            robotContainer::onRedAlliance,
            drivetrain
        );
    }

    /**
     * Named commands are how PathPlanner autos run robot code (intake, shoot, etc.).
     * The name here must match the name typed into the PathPlanner app exactly.
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("Do Nothing", doNothing());
        // Lock the wheels in an X at the end of an auto so we don't get pushed
        NamedCommands.registerCommand("X Brake", drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()).withTimeout(0.5));
    }

    public SendableChooser<Command> buildAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("Do Nothing", doNothing());
        // The string must match the .auto file name in deploy/pathplanner/autos (without ".auto")
        chooser.addOption("Drive Forward", AutoBuilder.buildAuto("Drive Forward"));
        return chooser;
    }

    /** Auton that does nothing */
    public Command doNothing() {
        return Commands.none();
    }
}
