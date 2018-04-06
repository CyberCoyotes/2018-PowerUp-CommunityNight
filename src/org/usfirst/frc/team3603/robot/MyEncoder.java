package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class MyEncoder implements PIDSource {

	WPI_TalonSRX talon;
	boolean inv;
	double multiplier;
	
	public MyEncoder(WPI_TalonSRX input, boolean invert, double mult) {
		talon = input;
		inv = invert;
		multiplier = mult;
		talon.getSensorCollection();
	}
	
	public void invert(boolean in) {
		inv = in;
	}
	
	public double get() {
		double distance = talon.getSelectedSensorPosition(0) * multiplier;
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
		talon.getSensorCollection().setPulseWidthPosition(0, 100);
		talon.getSensorCollection().setQuadraturePosition(0, 100);
		talon.getSensorCollection().setPulseWidthPosition(0, 100);
		talon.getSensorCollection().setQuadraturePosition(0, 100);
		talon.getSensorCollection().setPulseWidthPosition(0, 100);
		talon.getSensorCollection().setQuadraturePosition(0, 100);
		talon.getSensorCollection().setPulseWidthPosition(0, 100);
		talon.getSensorCollection().setQuadraturePosition(0, 100);
	}
	
	public void setMultiplier(double value) {
		multiplier = value;
	}

	@Override
	public double pidGet() {
		return get();
	}
}