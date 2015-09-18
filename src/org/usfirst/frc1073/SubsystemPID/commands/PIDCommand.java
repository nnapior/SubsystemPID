package org.usfirst.frc1073.SubsystemPID.commands;

public interface PIDCommand {
	public double getPIDSetpoint(int marker);
	public boolean isPIDEnabled();
	
}
