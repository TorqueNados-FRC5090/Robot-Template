// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TorqueNados - FRC 5090

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    // Log and replay timestamp and joystick data.
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        // DataLog + CTRE SignalLogger + power/CAN/command/event logs (USB stick in the RIO). Must be first.
        RobotLog.start(Constants.kDriveCANBus);
        m_robotContainer = new RobotContainer();
        SmartDashboard.putString("Code Version", readCodeVersion());
    }

    /** Branch + commit this code was built from (written by the writeVersionFile task in build.gradle) */
    private static String readCodeVersion() {
        try {
            return Files.readString(new File(Filesystem.getDeployDirectory(), "version.txt").toPath()).trim();
        } catch (IOException e) {
            DriverStation.reportWarning("Code Version unknown: " + e.getMessage(), false);
            return "unknown";
        }
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        // Runs every subsystem's periodic() (including Vision) and every scheduled command
        CommandScheduler.getInstance().run();
        m_robotContainer.updateBrownoutRumble();
        RobotLog.periodic();
    }

    /** One line at the start of auto and teleop, so every log says which match, alliance and auto it was. */
    private void logMatchStart(String phase) {
        RobotLog.event(String.format("%s START | %s match %d | %s alliance | auto: %s",
            phase, DriverStation.getMatchType(), DriverStation.getMatchNumber(),
            DriverStation.getAlliance().map(Enum::name).orElse("unknown"),
            m_autonomousCommand != null ? m_autonomousCommand.getName() : "none"));
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        logMatchStart("AUTO");
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        logMatchStart("TELEOP");
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
