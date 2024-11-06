package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.*;

// Subsystem Imports
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

// Command imports
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.wpilibj.DriverStation;

// Other imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_PORT);

    public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

    private final AutonContainer auton = new AutonContainer(this);
    private final SendableChooser<Command> autonChooser = auton.buildAutonChooser();

    /** Constructs a RobotContainer */
    public RobotContainer() {
        SmartDashboard.putData("Auton Selector", autonChooser);

        setDriverControls();
        setOperatorControls();
    }

    
    /** Configures a set of control bindings for the robot's driver */
    private void setDriverControls() {
        // Drives the robot with the joysticks
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, 
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightX()));
    }

    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {}
        
    
    /** @return Whether the robot is on the red alliance or not */
    public boolean onRedAlliance() { 
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    /** Use this to pass the autonomous command to the main {@link Robot} class. */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
