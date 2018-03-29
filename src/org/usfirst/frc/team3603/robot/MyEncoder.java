package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class MyEncoder implements PIDSource {

	WPI_TalonSRX talon;
	boolean inv;
	double multiplier;
	double offset;
	
	public MyEncoder(WPI_TalonSRX input, boolean invert, double mult) {
		talon = input;
		inv = invert;
		multiplier = mult;
		talon.getSensorCollection();
		offset = talon.getSelectedSensorPosition(0);
	}
	
	public void invert(boolean in) {
		inv = in;
	}
	
	public double get() {
		double distance = talon.getSelectedSensorPosition(0) * multiplier - offset;
		distance = inv ? -distance : distance;
		return distance;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}
	
	public void reset() {
		offset = talon.getSelectedSensorPosition(0);
	}
	
	public void setMultiplier(double value) {
		multiplier = value;
	}

	@Override
	public double pidGet() {
		return get();
	}
}