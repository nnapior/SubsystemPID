package org.usfirst.frc1073.SubsystemPID.subsystems;

public interface PIDSubsystem {
	public double getPIDSource(int marker);
	public void setPIDOutput(double output, int marker);
}
