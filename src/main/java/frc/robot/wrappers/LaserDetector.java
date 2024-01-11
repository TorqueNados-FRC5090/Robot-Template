package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

/** Wraps {@link DigitalInput} to represent a laser detector */
public class LaserDetector {
    private DigitalInput laser;

    /** 
     * Constructs LaserDetector object 
     * 
     * @param port The number of the DIO port being used
     */
    public LaserDetector(int port) {
        laser = new DigitalInput(port);
    }

    /** @return True if something is blocking the laser */
    public boolean isBlocked() { return !laser.get(); }

    /** @return True if nothing is blocking the laser */
    public boolean isOpen() { return laser.get(); }
}
