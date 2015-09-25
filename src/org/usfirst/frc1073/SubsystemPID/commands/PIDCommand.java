package org.usfirst.frc1073.SubsystemPID.commands;
/**
 * 
 * @author Derek Wider
 *	An interface to be implemented by any command that is to interact with a PIDThread.
 */
public interface PIDCommand {
	public double getPIDSetpoint(int marker);
	public boolean isPIDEnabled();
	
}
