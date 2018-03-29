package org.usfirst.frc.team3603.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.AnalogInput;

public class PressureSensor {
	
	AnalogInput input;
	public PressureSensor(int inputPin) {
		input = new AnalogInput(inputPin);
	}
	
	public double get() {
		DecimalFormat f = new DecimalFormat("#.0");
		double pressure = input.getVoltage() * 50.0 - 25.0;
		String s = f.format(pressure);
		return Double.parseDouble(s);
	}
	
	public double getRaw() {
		return input.getVoltage();
	}
}