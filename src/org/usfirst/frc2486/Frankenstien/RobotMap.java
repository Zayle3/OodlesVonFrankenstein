// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2486.Frankenstien;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SpeedController;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import java.util.Vector;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveTrainTalonFrontLeft;
    public static SpeedController driveTrainTalonRearLeft;
    public static RobotDrive driveTrainDriveLeft;
    public static SpeedController driveTrainTalonFrontRight;
    public static SpeedController driveTrainTalonRearRight;
    public static RobotDrive driveTrainDriveRight;
    public static CANTalon driveTraintalonCanLeft;
    public static CANTalon driveTraintalonCanRight;
    public static RobotDrive driveTrainCANDrive;
    public static Compressor pneumaticsCompressor;
    public static Solenoid pneumaticsSolenoid;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrainTalonFrontLeft = new Talon(2);
        LiveWindow.addActuator("Drive Train", "Talon Front Left", (Talon) driveTrainTalonFrontLeft);
        
        driveTrainTalonRearLeft = new Talon(1);
        LiveWindow.addActuator("Drive Train", "Talon Rear Left", (Talon) driveTrainTalonRearLeft);
        
        driveTrainDriveLeft = new RobotDrive(driveTrainTalonFrontLeft, driveTrainTalonRearLeft);
        
        driveTrainDriveLeft.setSafetyEnabled(false);
        driveTrainDriveLeft.setExpiration(0.1);
        driveTrainDriveLeft.setSensitivity(0.5);
        driveTrainDriveLeft.setMaxOutput(1.0);

        driveTrainTalonFrontRight = new Talon(3);
        LiveWindow.addActuator("Drive Train", "Talon Front Right", (Talon) driveTrainTalonFrontRight);
        
        driveTrainTalonRearRight = new Talon(0);
        LiveWindow.addActuator("Drive Train", "Talon Rear Right", (Talon) driveTrainTalonRearRight);
        
        driveTrainDriveRight = new RobotDrive(driveTrainTalonFrontRight, driveTrainTalonRearRight);
        
        driveTrainDriveRight.setSafetyEnabled(false);
        driveTrainDriveRight.setExpiration(0.1);
        driveTrainDriveRight.setSensitivity(0.5);
        driveTrainDriveRight.setMaxOutput(1.0);
        driveTrainDriveRight.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        driveTrainDriveRight.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        driveTraintalonCanLeft = new CANTalon(2);
        LiveWindow.addActuator("Drive Train", "talonCanLeft", driveTraintalonCanLeft);
        
        driveTraintalonCanRight = new CANTalon(1);
        LiveWindow.addActuator("Drive Train", "talonCanRight", driveTraintalonCanRight);
        
        driveTrainCANDrive = new RobotDrive(driveTraintalonCanLeft, driveTraintalonCanRight);
        
        driveTrainCANDrive.setSafetyEnabled(false);
        driveTrainCANDrive.setExpiration(0.1);
        driveTrainCANDrive.setSensitivity(0.5);
        driveTrainCANDrive.setMaxOutput(1.0);

        pneumaticsCompressor = new Compressor(13);
        
        
        pneumaticsSolenoid = new Solenoid(13, 0);
        LiveWindow.addActuator("Pneumatics", "Solenoid", pneumaticsSolenoid);
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
