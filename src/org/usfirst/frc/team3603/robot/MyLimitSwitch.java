package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class MyLimitSwitch {

	WPI_TalonSRX talon;
	Position position;
	
	public MyLimitSwitch(WPI_TalonSRX tal, Position pos) {
		talon = tal;
		position = pos;
	}
	
	public boolean get() {
		return position == Position.fwd ? talon.getSensorCollection().isFwdLimitSwitchClosed() : talon.getSensorCollection().isRevLimitSwitchClosed();
	}
	
	public enum Position {
		fwd, rev
	}
}
