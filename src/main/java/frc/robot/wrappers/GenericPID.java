package frc.robot.wrappers;

// Imports
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase;

/** Wraps over {@link SparkPIDController} for ease of use */
public class GenericPID {
    /** The motor being controlled */
    private CANSparkBase motor;
    /** The motor's PID controller */
    private SparkPIDController controller;

    /** Proportional gain */
    private double P; 
    /** Integral gain */
    private double I; 
    /** Derivative gain */
    private double D;

    /** The PID controller's target */
    private double setpoint = 0;
    /** {@link CANSparkMax.ControlType How} the motor should be controlled */
    private CANSparkBase.ControlType controlType;

    /** The minimum setpoint to be allowed */
    private double min = Integer.MIN_VALUE;
    /** The maximum setpoint to be allowed */
    private double max = Integer.MAX_VALUE;

    // NOTE: ratio is typically used by position controllers to covert
    // from revolutions to a more useful unit.
    /** Incoming setpoint related values will be multiplied by this.
      * Outgoing setpoint related values will be divided by this. */
    private double ratio = 1;

    /** Constructs a GenericPID object 
     *  @param motor The {@link CANSparkMax motor} to control 
     *  @param controlType {@link CANSparkMax.ControlType How} the motor should be controlled
     *  @param P The P value to be used by the controller */
    public GenericPID(CANSparkBase motor, CANSparkBase.ControlType controlType, double P) {
        this(motor, controlType, P, 0, 0);
    }

    /** Constructs a GenericPID object 
     *  @param motor The {@link CANSparkMax motor} to control 
     *  @param controlType {@link CANSparkMax.ControlType How} the motor should be controlled
     *  @param P The P value to be used by the controller 
     *  @param I The I value to be used by the controller 
     *  @param D The D value to be used by the controller */
    public GenericPID(CANSparkBase motor, CANSparkBase.ControlType controlType, double P, double I, double D) {
        this.motor = motor;
        controller = motor.getPIDController();

        this.controlType = controlType;

        this.P = P;
        controller.setP(P);
        this.I = I;
        controller.setI(I);
        this.D = D;
        controller.setD(D);
    }


    // Accessor methods
    public double getP() { return controller.getP(); }
    public double getI() { return controller.getI(); }
    public double getD() { return controller.getD(); }
    public double getSetpoint() { return usingPositionControl() ? setpoint/ratio : setpoint; }
    public double getRatio(){ return ratio; }
    public CANSparkBase.ControlType getControlType() { return controlType; }
    public double getMin() { return min; }
    public double getMax() { return max; }
    /** Calculates whether the motor has reached its setpoint based off of the set control type
     *  @param tolerance How far the actual value is allowed to be from the setpoint
     *  @return Whether the PID has reached its setpoint */
    public boolean atSetpoint(double tolerance) { return Math.abs(getMeasurement() - getSetpoint()) <= tolerance; }
    /** @return The measured value from the motor based on the set {@link CANSparkMax.ControlType control type}. */
    public double getMeasurement() {
        switch(controlType) {
            case kDutyCycle: return motor.getAppliedOutput();
            case kVelocity: return motor.getEncoder().getVelocity();
            case kVoltage: return motor.getAppliedOutput() * motor.getBusVoltage();
            case kPosition: return motor.getEncoder().getPosition() / ratio;
            case kSmartMotion: return motor.getEncoder().getPosition();
            case kCurrent: motor.getOutputCurrent();
            case kSmartVelocity: return motor.getEncoder().getVelocity();
            default: return 0;
        }
    }

    private boolean usingPositionControl() {return controlType.equals(ControlType.kPosition); }

    // Setter Methods
    public void setP(double P) { this.P = P; controller.setP(P); }
    public void setI(double I) { this.I = I; controller.setI(I); }
    public void setD(double D) { this.D = D; controller.setD(D); }
    public void setPID(double P, double I, double D) { 
        this.P = P; controller.setP(P);
        this.I = I; controller.setI(I);
        this.D = D; controller.setD(D); 
    }
    public void setControlType(CANSparkBase.ControlType controlType) { this.controlType = controlType; }
    /** @param ratio Incoming position instructions are multiplied by this, outgoing ones are divided */
    public void setRatio(double ratio){ this.ratio = ratio;}
    public void setMin(double min) { this.min = min*ratio; setSetpoint(this.setpoint); }
    public void setMax(double max) { this.max = max*ratio; setSetpoint(this.setpoint); }
    /** Set the min and max input values */
    public void setInputRange(double min, double max) {setMin(min); setMax(max); setSetpoint(this.setpoint); }
    /** Set the min and max output speed [-1,1] */
    public void setOutputRange(double min, double max) { controller.setOutputRange(min, max); }
    
    /** Sets the setpoint and forces it within user-set bounds [min,max] */
    public void setSetpoint(double set) {
        if (usingPositionControl()) set *= ratio;
        
        this.setpoint = set < min ? min : 
                      ( set > max ? max : set );
    }

    /** Set the PID gains to match the object settings */
    public void updatePID() { 
        if( this.P != controller.getP())
            controller.setP(this.P);

        if( this.I != controller.getI())
            controller.setI(this.I);

        if( this.D != controller.getD())
            controller.setD(this.D);
    }

    /** Activate the PID controller using the internal setpoint */
    public void activate() { activate(this.setpoint); }
    /** Activate the PID controller
     *  @param setpoint The PID controller's target */
    public void activate(double setpoint) {
        updatePID();
        setSetpoint(setpoint);
        controller.setReference(this.setpoint, this.controlType);
    }

    /** Sets the PID gains to 0, without changing the stored values */
    public void pause(){
        controller.setP(0);
        controller.setI(0);
        controller.setD(0);
    }

    /** Sets the PID gains to 0, as well as the stored values */
    public void stop() {
        this.setP(0);
        this.setI(0);
        this.setD(0);
    }
}
