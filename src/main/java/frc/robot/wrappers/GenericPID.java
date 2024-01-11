package frc.robot.wrappers;

// Imports
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;

/** Wraps over {@link SparkPIDController} for ease of use */
public class GenericPID {
    /** The motor being controlled */
    private CANSparkMax motor;
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
    private CANSparkMax.ControlType controlType;

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
    public GenericPID(CANSparkMax motor, CANSparkMax.ControlType controlType, double P) {
        this( motor, controlType, P, 0, 0, 1);
    }

    /** Constructs a GenericPID object 
     *  @param motor The {@link CANSparkMax motor} to control 
     *  @param controlType {@link CANSparkMax.ControlType How} the motor should be controlled
     *  @param P The P value to be used by the controller 
     *  @param ratio Incoming setpoint related values will be multiplied by this.
     *               Outgoing setpoint related values will be divided by this. */
    public GenericPID(CANSparkMax motor, CANSparkMax.ControlType controlType, double P, double ratio) {
        this( motor, controlType, P, 0, 0, ratio);
    }

    /** Constructs a GenericPID object 
     *  @param motor The {@link CANSparkMax motor} to control 
     *  @param controlType {@link CANSparkMax.ControlType How} the motor should be controlled
     *  @param P The P value to be used by the controller 
     *  @param I The I value to be used by the controller 
     *  @param D The D value to be used by the controller */
    public GenericPID(CANSparkMax motor, CANSparkMax.ControlType controlType, double P, double I, double D) {
        this( motor, controlType, P, I, D, 1);
    }

    /** Constructs a GenericPID object 
     *  @param motor The {@link CANSparkMax motor} to control 
     *  @param controlType {@link CANSparkMax.ControlType How} the motor should be controlled
     *  @param P The P value to be used by the controller 
     *  @param I The I value to be used by the controller 
     *  @param D The D value to be used by the controller 
     *  @param ratio Incoming setpoint related values will be multiplied by this.
     *               Outgoing setpoint related values will be divided by this. */
    public GenericPID(CANSparkMax motor, CANSparkMax.ControlType controlType, double P, double I, double D, double ratio) {
        this.motor = motor;
        controller = motor.getPIDController();

        this.controlType = controlType;

        this.P = P;
        controller.setP(P);
        this.I = I;
        controller.setI(I);
        this.D = D;
        controller.setD(D);

        this.ratio = ratio;
    }




    // Accessor methods
    public double getRatio(){ return ratio;}
    public double getP() { return controller.getP(); }
    public double getI() { return controller.getI(); }
    public double getD() { return controller.getD(); }
    public double getSetpoint() { return setpoint/ratio; }
    public CANSparkMax.ControlType getControlType() { return controlType; }
    public double getRPM() { return motor.getEncoder().getVelocity(); }
    public double getPositionNoRatio() { return motor.getEncoder().getPosition(); }
    public double getPosition() {return getPositionNoRatio()/ratio;}
    public double getMin() { return min; }
    public double getMax() { return max; }
    public SparkPIDController getController() { return controller; }
    public CANSparkMax getMotor() { return motor; }

    // Setter Methods
    public void setP(double P) { this.P = P; controller.setP(P); }
    public void setI(double I) { this.I = I; controller.setI(I); }
    public void setD(double D) { this.D = D; controller.setD(D); }
    public void setPID(double P, double I, double D) { 
        this.P = P; controller.setP(P);
        this.I = I; controller.setI(I);
        this.D = D; controller.setD(D); 
    }
    public void setControlType(CANSparkMax.ControlType controlType) { this.controlType = controlType; }
    public void setRatio(double ratio){ this.ratio = ratio;}
    public void setMin(double min) { this.min = min*ratio; setSetpoint(this.setpoint); }
    public void setMax(double max) { this.max = max*ratio; setSetpoint(this.setpoint); }
    /** Set the min and max input values */
    public void setInputRange(double min, double max) {setMin(min); setMax(max); setSetpoint(this.setpoint); }
    /** Set the min and max output speed [-1,1] */
    public void setOutputRange(double min, double max) { controller.setOutputRange(min, max); }
    
    /** Sets the setpoint, and forces it within user-set bounds [min,max] */
    public void setSetpoint(double set) {
        set *= ratio;

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
