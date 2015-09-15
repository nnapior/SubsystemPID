package org.usfirst.frc1073.SubsystemPID;
	
public class PIDThread extends Thread{
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

	public PIDThread(double kP, double kI, double kD, long dt, double tolerance){
		this.kP = kP;
		this.kI = kI;
		this.kD = kP;
		this.dt = dt;
		this.tolerance = tolerance;
		this.setpoint = 0;
		this.currentMeasurement = 0;
	}
	public void run(){
		
		//PID base code below:
		double error = (setpoint) - (currentMeasurement);
		error = toleranceAdjustment(error);
		integral = integral + (error * dt);
		double derivative = (error - previousError) / dt;
		output = (kP * error) + (kI * integral) + (kD * derivative);
		previousError = error;
		
		try {
			Thread.sleep(dt);
		} catch (InterruptedException e) {
			System.out.println("PIDThread interupted. Switch to manual control");
		}
	}

	private double toleranceAdjustment(double currentError){
		if(Math.abs(currentError) > tolerance){
			return currentError;
		}
		return 0.0;
	}
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
