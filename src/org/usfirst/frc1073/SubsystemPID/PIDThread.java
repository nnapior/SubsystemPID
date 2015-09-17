package org.usfirst.frc1073.SubsystemPID;

import org.usfirst.frc1073.SubsystemPID.commands.PIDCommand;
import org.usfirst.frc1073.SubsystemPID.subsystems.PIDSubsystem;
	
public class PIDThread implements Runnable{
	private double kP;
	private double kI;
	private double kD;
	private long dt;
	private double tolerance;

	private double previousError;
	private double integral;
	private double output;
	private double setpoint;
	private double currentMeasurement;
	
	private PIDSubsystem PIDinput;
	private PIDSubsystem PIDOutput;
	private PIDCommand PIDSetpoint;

	private boolean disable;
	private int marker; //<--- this is SUPER IMPORTANT. Each PID Thread needs its own unique marker, or it will NOT WORK
	public PIDThread(double kP, double kI, double kD, long dt, double tolerance, int marker){
		this.kP = kP;
		this.kI = kI;
		this.kD = kP;
		this.dt = dt;
		this.tolerance = tolerance;
		this.setpoint = 0;
		this.currentMeasurement = 0;
		this.PIDinput = null;
		this.PIDOutput =  null;
		this.PIDSetpoint = null;
		this.marker = marker;
		disable = false;
	}
	public void run(){
		
		while (!disable) { //this needs to be changed to know when to quit out. 
			setpoint = PIDSetpoint.getPIDSetpoint(marker);
			currentMeasurement = PIDinput.getPIDSource(marker);

			// PID base code below:
			double error = (setpoint) - (currentMeasurement);
			error = toleranceAdjustment(error);
			integral = integral + (error * dt);
			double derivative = (error - previousError) / dt;
			output = (kP * error) + (kI * integral) + (kD * derivative);
			
			previousError = error;

			PIDOutput.setPIDOutput(output, marker);
			
			
			try {
				Thread.sleep(dt);
			} catch (InterruptedException e) {
				System.out.println("PIDThread #" + marker + "interupted");
			}
		}

	}

	private double toleranceAdjustment(double currentError){
		if(Math.abs(currentError) > tolerance){
			return currentError;
		}
		return 0.0;
	}
	public void setPIDObjects(PIDSubsystem PIDinput, PIDSubsystem PIDOutput, PIDCommand PIDSetpoint){
		this.PIDinput = PIDinput;
		this.PIDOutput =  PIDOutput;
		this.PIDSetpoint = PIDSetpoint;
	}
	
	//unused
	public void updateSetpoint(double newSetpoint){
		this.setpoint = newSetpoint;
	}
	public void updateCurrentMeasurement(double newMeasurement){
		this.currentMeasurement = newMeasurement;
	}
	public double getOutput(){
		return output;
	}
}
