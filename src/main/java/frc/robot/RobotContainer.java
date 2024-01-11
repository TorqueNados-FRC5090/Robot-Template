package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.*;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonContainer;

// Other imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final XboxController driverController = new XboxController(DRIVER_PORT);
    private final XboxController operatorController = new XboxController(OPERATOR_PORT);
    private final AutonContainer auton = new AutonContainer();
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    /** Constructs a RobotContainer */
    public RobotContainer() {
        initChooser();
    }

    /** Initialize the auton selector on the dashboard */
    private void initChooser() {
        SmartDashboard.putData("Auton Selector", autonChooser);
        autonChooser.setDefaultOption("Do Nothing", auton.doNothing());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
