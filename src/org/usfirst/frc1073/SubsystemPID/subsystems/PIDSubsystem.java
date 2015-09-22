package org.usfirst.frc1073.SubsystemPID.subsystems;
/**
* An interface for use with PIDThread. Any subsystem that will be using a PIDThread must implement this interface
*/
public interface PIDSubsystem {
	public double getPIDSource(int marker);
	public void setPIDOutput(double output, int marker);
}
