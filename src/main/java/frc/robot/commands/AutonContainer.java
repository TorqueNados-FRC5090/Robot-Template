package frc.robot.commands;

import static frc.robot.Constants.SwerveConstants.MAX_TRANSLATION_SPEED;
import static frc.robot.Constants.SwerveConstants.ModuleConstants.WHEEL_DIAMETER;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer {
    private SwerveDrivetrain drivetrain;

    /** Constructs an AutonContainer object */ 
    public AutonContainer(RobotContainer robot) {
        this.drivetrain = robot.drivetrain;
        registerNamedCommands();

        AutoBuilder.configureHolonomic(
            drivetrain::getPoseMeters, 
            drivetrain::setOdometry,
            drivetrain::getChassisSpeeds,
            drivetrain::driveRobotRelative,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    MAX_TRANSLATION_SPEED, // Max module speed, in m/s
                    WHEEL_DIAMETER, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> robot.onRedAlliance(),
            drivetrain);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("Do Nothing", doNothing() );
    }

    public SendableChooser<Command> buildAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("Do Nothing", doNothing());
        return chooser;
    }

    /** Auton that does nothing */
    public Command doNothing() {
        return new WaitCommand(0);
    }
}