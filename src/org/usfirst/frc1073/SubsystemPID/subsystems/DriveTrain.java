// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc1073.SubsystemPID.subsystems;

import org.usfirst.frc1073.SubsystemPID.RobotMap;
import org.usfirst.frc1073.SubsystemPID.commands.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class DriveTrain extends Subsystem implements PIDSubsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SpeedController frontLeftJag = RobotMap.driveTrainfrontLeftJag;
    SpeedController frontRightJag = RobotMap.driveTrainfrontRightJag;
    SpeedController backLeftJag = RobotMap.driveTrainbackLeftJag;
    SpeedController backRightJag = RobotMap.driveTrainbackRightJag;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    public Drive drive;
    private boolean isPID;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
    	drive = new Drive();
    	setDefaultCommand(drive);
    	//System.out.println("In drivetrain default command");
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    public void setFrontLeftSpeed(double speed){
    	frontLeftJag.set(speed);
    }
    public void setFrontRightSpeed(double speed){
    	frontRightJag.set(speed);
    }
    public void setBackLeftSpeed(double speed){
    	backLeftJag.set(speed);
    }
    public void setBackRightSpeed(double speed){
    	backRightJag.set(speed);
    }
	@Override
	public double getPIDSource(int marker) {
		return 0;
	}
	@Override
	public void setPIDOutput(double output, int marker) {
		if (isPID) {
			if (marker == 0) {
				frontLeftJag.set(output);
			} else if (marker == 1) {
				frontRightJag.set(output);
			} else if (marker == 2) {
				backLeftJag.set(output);
			} else if (marker == 3) {
				backRightJag.set(output);
			}
		}
	}
	public Drive getDriveCommand(){
		return drive;
	}
	public void enablePIDDrivetrainParameter(){
		isPID = true;
	}
	public void disablePIDDrivetrainParameter(){
		isPID = false;
	}
	
}

