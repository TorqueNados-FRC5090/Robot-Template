package frc.robot.subsystems.drivetrain;

// Motor imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class represents the drivetrain of a standard west-coast robot */
public class WestCoastDrivetrain extends SubsystemBase {
    private CANSparkMax leftMotor;
    private CANSparkMax leftMotorFollower;
    private CANSparkMax rightMotor;
    private CANSparkMax rightMotorFollower;


    /** Constructs a WestCoastDrivetrain object using the motor 
     *  IDs provided by {@link frc.robot.Constants.WestCoastIDs} */
    public WestCoastDrivetrain(int leftMotorLeaderID, int leftMotorFollowerID,
                               int rightMotorLeaderID, int rightMotorFollowerID) {
        
        leftMotor = new CANSparkMax(leftMotorLeaderID, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotorFollower = new CANSparkMax(leftMotorFollowerID, MotorType.kBrushless);
        leftMotorFollower.restoreFactoryDefaults();
        leftMotorFollower.follow(leftMotor);
        
        rightMotor = new CANSparkMax(rightMotorLeaderID, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotorFollower = new CANSparkMax(rightMotorFollowerID, MotorType.kBrushless);
        rightMotorFollower.restoreFactoryDefaults();
        rightMotorFollower.follow(rightMotor);
    }

    /** Drives the robot with standard arcade controls where one 
     *  axis controls speed and the other controls rotation */
    public void arcadeDrive(double drive, double turn) {
        leftMotor.set(drive + turn);
        rightMotor.set(drive - turn);
    }
}
