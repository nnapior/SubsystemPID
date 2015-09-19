package org.usfirst.frc1073.SubsystemPID;

import org.usfirst.frc1073.SubsystemPID.commands.PIDCommand;
import org.usfirst.frc1073.SubsystemPID.subsystems.PIDSubsystem;
	/**
	 * 
	 * @author Derek Wider
	 * FIRST Team 1073
	 * 
	 * A runnable, generic PID class. 
	 *
	 */
public class PIDThread implements Runnable{
	//PID constants
	private double kP;
	private double kI;
	private double kD;
	//thread refresh rate - recommended 5ms (200hz)
	private long dt;
	//tolerance specifies the "close enough factor" - when the error is within +- this tolerance, error is considered 0
	private double tolerance;
	//PID function variables
	private double previousError;
	private double integral;
	private double output;
	private double setpoint;
	private double currentMeasurement;
	//PID interface objects - input (sensor), output (motor controller or other), and setpoint (joystick or raw data point)
	private PIDSubsystem PIDinput;
	private PIDSubsystem PIDOutput;
	private PIDCommand PIDSetpoint;
	
	private boolean enabled;
	private int marker; //<--- this is SUPER IMPORTANT. Each PID Thread needs its own unique marker, or it will NOT WORK
	/**
	 * Constructs a PID Thread
	 * @param kP - the Proportional parameter
	 * @param kI - the integral parameter
	 * @param kD - the derivative parameter
	 * @param dt - thread refresh rate (5ms recommended)
	 * @param tolerance - error considered 0 if error within +- this value
	 * @param marker - VERY IMPORTANT - each new PID thread must have its own PID marker (0 through 3 reserved for PID Drive)
	 */
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
		enabled = true;
	}
	/**
	 * 
	 */
	public void run(){
		
		while (true) { 
			if(enabled){
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
			}
			else{
				PIDOutput.setPIDOutput(0.0, marker);
				integral = 0;
			}
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
	
	public void disable(){
		enabled = false;
	}
	public void enable(){
		enabled = true;
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
