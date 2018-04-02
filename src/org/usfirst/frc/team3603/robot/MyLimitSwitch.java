package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

public class MyLimitSwitch {

	DigitalInput dio;
	
	public MyLimitSwitch(int pin) {
		dio = new DigitalInput(pin);
	}
	
	public boolean get() {
		return dio.get();
	}
}
