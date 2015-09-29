package org.usfirst.frc1073.SubsystemPID;

import org.usfirst.frc1073.SubsystemPID.commands.PIDCommand;
import org.usfirst.frc1073.SubsystemPID.subsystems.PIDSubsystem;
	/**
	 * 
	 * @author Derek Wider
	 * FIRST Team 1073
	 * 
	 * A runnable, generic PID class. Runs on a separate thread and all system control can be re-routed around a PIDthread should something go wrong. 
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
	 * Constructs a PID Thread. Each PID thread is an implementation of the Runnable interface. PID Threads can be created or stopped anytime, and can also be enabled/disabled ("Paused") when not in use. To use a PID Thread,
	   you'll need 3 PID-specific objects - 2 PIDSubsystems and a PIDCommand. PIDSubsystem and PIDCommand are interfaces that your subsystems and commands should implement if they are to use PID. If you're looking for more information about how PID works, 
	   please refer to this introductory piece: http://www.societyofrobots.com/programming_PID.shtml
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
	 * Called when the thread the PIDThread is passed into is started (myThread.start())
	 */
	public void run(){
		//TODO add sensor failure detection to abort loop and return to manual/variable voltage control
		while (true) { 
			//skip PID if the thread is disabled
			if(enabled){
				//Core PID Code follows: 
				//setpoint get the setpoint from the PIDSetpoint passed in (use this thread's marker)
				//currentMeasurement gets the current reading from the PIDinput (use this thread's marker)
				setpoint = PIDSetpoint.getPIDSetpoint(marker);
				currentMeasurement = PIDinput.getPIDSource(marker);
				//PID Calculation (includes tolerance adjustment)
				double error = (setpoint) - (currentMeasurement);
				error = toleranceAdjustment(error);
				integral = integral + (error * dt);
				double derivative = (error - previousError) / dt;
				output = (kP * error) + (kI * integral) + (kD * derivative);
				previousError = error;
				//set the PIDoutput to the generated output (again, use the specific marker to prevent cross-thread data transmission, ex. left front encoder reading used in right front PID)
				PIDOutput.setPIDOutput(output, marker);
			}
			//if PID disabled, just set output to 0 and 0 integral.
			//IMPORTANT - if switching to manual control, implement a "disregard all data" catch in the setPIDOutput() method of your PIDOutput object to prevent 0 movement from actuator. 
			else{
				PIDOutput.setPIDOutput(0.0, marker);
				integral = 0;
			}
			//thread frequency adjustment. catch statement can also be used to trigger events
			try {
				Thread.sleep(dt);
			} catch (InterruptedException e) {
				System.out.println("PIDThread #" + marker + "interupted");
			}
		}

	}
	/**
	* Adjusts the error for the specified tolerance
	* @param currentError - the current PID error
	*/
	private double toleranceAdjustment(double currentError){
		if(Math.abs(currentError) > tolerance){
			return currentError;
		}
		return 0.0;
	}
	/**
	* Establishes the three PID objects used in the core calculation (each object implements either PIDSubsystem or PIDCommand)
	* @param PIDinput - the subsystem containing the sensor used in the PID calculation
	* @param PIDOutput - the subsystem containing the output device (motor controller, relay, etc) used in the PID calculation
	* @param PIDSetpoint - The command that gets the setpoint used in PID. This can be any command, as long as it specifies a setpoint (can be joystick value, autonomous distance, etc)
	*/
	public void setPIDObjects(PIDSubsystem PIDinput, PIDSubsystem PIDOutput, PIDCommand PIDSetpoint){
		this.PIDinput = PIDinput;
		this.PIDOutput =  PIDOutput;
		this.PIDSetpoint = PIDSetpoint;
	}
	/**
	* Disables PID calculations (does NOT stop thread execution)
	*/
	public void disable(){
		enabled = false;
	}
	/**
	* Enables PID calculations (thread is still running, data retrieval simply re-initiated)
	* Also can be used to reset PID algorithm if integral has wound up
	**/
	public void enable(){
		enabled = true;
	}
	
	//unused methods but kept in for future reference
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
