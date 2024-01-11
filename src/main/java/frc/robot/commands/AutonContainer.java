package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer {

    /** Constructs an AutonContainer object */ 
    public AutonContainer() {}


    /** Auton that drops a piece high, reverses, and sets heading*/
    public Command doNothing() {
        return new WaitCommand(0);
    }
}
