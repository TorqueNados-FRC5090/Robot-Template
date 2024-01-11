package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalInput;

/** Wraps {@link DigitalInput} to represent a laser detector */
public class LimitSwitch {
    private DigitalInput limSwitch;

     /** 
     * Constructs LimitSwitch object 
     * 
     * @param port The number of the DIO port being used
     */
    public LimitSwitch(int port) {
        limSwitch = new DigitalInput(port);
    }

    /** @return true if the switch is being pressed */
    public boolean isPressed() { return !limSwitch.get(); }

    /** @return true if the switch is not being pressed */
    public boolean isNotPressed() { return limSwitch.get(); }
}