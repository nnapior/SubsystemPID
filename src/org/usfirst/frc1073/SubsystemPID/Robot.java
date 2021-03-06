// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc1073.SubsystemPID;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc1073.SubsystemPID.commands.*;
import org.usfirst.frc1073.SubsystemPID.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    Command autonomousCommand;
    
    //PID Constants - used for the 4 PID drive threads
    private double kP;
    private double kI;
    private double kD;
    
    //tolerance - specifies when the 4 drive threads are "close enough" to the setpoint
    private double tolerance;
    
    //dt - specifies refresh rate of the PID drive threads
    private long dt;
    
    //isFirstStartup - starts the threads on startup, prevents starting threads after they have already been started
    private boolean isFirstStartup;
    
    public static OI oi;
    
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static DriveTrain driveTrain;
    public static EncoderSystem encoderSystem;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    //The actual threads that the PIDThreads are passed into.
    Thread frontLeftThread;
    Thread frontRightThread;
    Thread backLeftThread;
    Thread backRightThread;
    
    //the 4 PID drive threads (implement the runnable interface)
    PIDThread frontLeftPIDThread;
    PIDThread frontRightPIDThread;
    PIDThread backLeftPIDThread;
    PIDThread backRightPIDThread;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	RobotMap.init();
    
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrain = new DriveTrain();
        encoderSystem = new EncoderSystem();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands 
        //(which it very likely will), subsystems are not guaranteed to be 
        // constructed yet. Thus, their requires() statements may grab null 
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // instantiate the command used for the autonomous period
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        autonomousCommand = new AutonomousCommand();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        
        //default PID constants for drive
        kP = 0.05;
        kI = 0.05;
        kD = 0.05;
        
        //default tolerance: +- 1% of setpoint
        tolerance = 0.01;
        
        //default refresh rate: 5ms (or 200hz)
        dt = 5;
        
        //the drive PID threads - set up with defaults and their respective markers
        frontLeftPIDThread = new PIDThread(kP, kI, kD, dt, tolerance, 0);
        frontRightPIDThread = new PIDThread(kP, kI, kD, dt, tolerance, 1);
        backLeftPIDThread = new PIDThread(kP, kI, kD, dt, tolerance, 2);
        backRightPIDThread = new PIDThread(kP, kI, kD, dt, tolerance, 3);
        
        //pass the PID runnable objects into the threads
        frontLeftThread = new Thread(frontLeftPIDThread);
        frontRightThread = new Thread(frontRightPIDThread);
        backLeftThread = new Thread(backLeftPIDThread);
        backRightThread = new Thread(backRightPIDThread);
        
        //specify that this is the first startup
        isFirstStartup = true;
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){
    	//disables the threads when the robot is disabled (method of the PIDThread class)
    	frontLeftPIDThread.disable();
    	frontRightPIDThread.disable();
    	backLeftPIDThread.disable();
    	backRightPIDThread.disable();
    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
		//switches if it is/isn't first startup. Prevents starting a thread multiple times, which leads to a IllegalThreadStateException
        if (isFirstStartup) {
        	//specifies the units of the encoders
        	Robot.encoderSystem.setUnitScale(); 
			//Very important: below code specifies the PID sources and commands for each thread 
			frontLeftPIDThread.setPIDObjects(encoderSystem, driveTrain,
					Robot.driveTrain.getDriveCommand());
			frontRightPIDThread.setPIDObjects(encoderSystem, driveTrain,
					Robot.driveTrain.getDriveCommand());
			backLeftPIDThread.setPIDObjects(encoderSystem, driveTrain,
					Robot.driveTrain.getDriveCommand());
			backRightPIDThread.setPIDObjects(encoderSystem, driveTrain,
					Robot.driveTrain.getDriveCommand());
			//starts PID threads
			frontLeftThread.start();
			frontRightThread.start();
			backLeftThread.start();
			backRightThread.start();
			
			isFirstStartup = false;
		}
        
        //if the robot has been enabled before, runs the enable() method instead of start().
		frontLeftPIDThread.enable();
		frontRightPIDThread.enable();
		backLeftPIDThread.enable();
		backRightPIDThread.enable();
        
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
  
}
