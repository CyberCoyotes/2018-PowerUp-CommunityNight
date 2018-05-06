package org.usfirst.frc.team3603.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	WPI_TalonSRX left1 = new WPI_TalonSRX(5);
	WPI_TalonSRX left2 = new WPI_TalonSRX(6);
	WPI_TalonSRX right1 = new WPI_TalonSRX(3);
	WPI_TalonSRX right2 = new WPI_TalonSRX(4);
	SpeedControllerGroup left = new SpeedControllerGroup(left1, left2);
	SpeedControllerGroup right = new SpeedControllerGroup(right1, right2);
	DifferentialDrive mainDrive = new DifferentialDrive(left, right);
	
	WPI_TalonSRX leftHolder = new WPI_TalonSRX(9);//Leftholder speedcontroller
	WPI_TalonSRX rightHolder = new WPI_TalonSRX(8);//Rightholder speedcontroller 
	WPI_TalonSRX cubeLift = new WPI_TalonSRX(1); //Cube lift speed controller
	WPI_TalonSRX lift2 = new WPI_TalonSRX(2);//Second lift controller
	WPI_TalonSRX arm = new WPI_TalonSRX(7); //Arm speed controller
	Servo release = new Servo(0); //Servo for the arm release
	
	//Compressor compressor = new Compressor(); //Air compressor
	DoubleSolenoid shift = new DoubleSolenoid(2, 3);//Transmission solenoid
	DoubleSolenoid grabber = new DoubleSolenoid(0, 1);//TODO check to see if this is correct
	
	Joystick joy1 = new Joystick(0); //Large twist-axis joystick
	Joystick joy2 = new Joystick(1); //Xbox controller
	
	Encoder armEnc = new Encoder(0, 1, false, EncodingType.k2X); //Arm angle encoder
	MyEncoder liftEnc = new MyEncoder(cubeLift, false, 1.0); //Encoder for the cube lift
	
	PressureSensor pressure = new PressureSensor(0); //Pressure sensor
	AHRS gyro = new AHRS(Port.kMXP); //NavX
	
	PIDController strPID = new PIDController(0.05, 0, 0, gyro, new Spark(7)); //PID controller for driving straight
	PIDController liftPID = new PIDController(0.001, 0, 0, liftEnc, cubeLift); //PID controller for lift
	PIDController armPID = new PIDController(0.08, 0, 0, armEnc, arm); //PID controller for arm
	
	DigitalInput slot1 = new DigitalInput(2); //Digital inputs for the auton switch
	DigitalInput slot2 = new DigitalInput(3);
	DigitalInput slot3 = new DigitalInput(4);
	DigitalInput highSwitch = new DigitalInput(5);
	DigitalInput lowSwitch = new DigitalInput(6);
	
	DriverStation matchInfo = DriverStation.getInstance(); //Object to get switch/scale colors

	final static DoubleSolenoid.Value out = DoubleSolenoid.Value.kForward; //Piston out value
	final static DoubleSolenoid.Value in = DoubleSolenoid.Value.kReverse; //Piston in value
	Compressor compressor = new Compressor();
	@Override
	public void robotInit() {
		compressor.start();
		cubeLift.setInverted(true);//TODO invert if the PID doesn't work
		lift2.setInverted(true);//TODO invert if the PID doesn't work
		lift2.set(ControlMode.Follower, 1);
		
		left.setInverted(true);
		right.setInverted(true);
		rightHolder.setInverted(true);
		
		cubeLift.getSensorCollection();//TODO remove?
		mainDrive.setSafetyEnabled(false); //Disable safety
		
		liftPID.setOutputRange(-0.7, 0.7); //Set the range of speeds for the lift PID
		armPID.setOutputRange(-0.5, 0.5); //Set the range of speeds for the arm PID
		liftEnc.reset(); //Zero out the lift 
	}
	
	@Override
	public void disabledPeriodic() {
		read();
	}
	
	@Override
	public void autonomousInit() {
	}//TODO add pneumatics
	
	@Override
	public void autonomousPeriodic() {
	}
	
	@Override
	public void teleopInit() {
		armPID.setSetpoint(armEnc.get());
		liftPID.setSetpoint(liftEnc.get());
		liftPID.setOutputRange(-0.7, 0.7);
		liftPID.disable();
		shift.set(in);
		strPID.disable();
    	compressor.start();
	}
	
	boolean pistonBool = false;
	boolean manual = true;
	boolean doOnce = false;
	
	@Override
	public void teleopPeriodic() {
		release.set(0.5);
		
		/**********
		 * DRIVER *
		 **********/
		
		double sense = -0.5 * joy1.getRawAxis(3) + 0.5;//Sensitivity coefficient
		double y = -Math.pow(joy1.getRawAxis(1), 1); //Double to store the joystick's y axis
		double rot = Math.pow(joy1.getRawAxis(2), 1)*(9.0/10.0); //Double to store the joystick's x axis
		if(Math.abs(y) >= 0.05 || Math.abs(rot) >= 0.05 && !joy1.getRawButton(1)) { //Thresholding function
			mainDrive.arcadeDrive(y * sense, rot * sense); //Arcade drive with the joystick's axis
		} else {
			mainDrive.arcadeDrive(0, 0); //Stop if value doesn't meet threshhold
		}
		
		if(joy1.getRawButton(3)) { //Press and hold button 3 for transmission
			shift.set(out);//Set the transmission piston to out (high gear)
		} else {
			shift.set(in); //Set the transmission piston to in (low gear)
		}
		
		
		/***************
		 * MANIPULATOR *
		 ***************/
		if(joy2.getRawButtonPressed(1)) {
			pistonBool = !pistonBool;
		}
		if(pistonBool) {
			grabber.set(out);
		} else {
			grabber.set(in);
		}
		
		if(Math.abs(joy2.getRawAxis(1)) >= 0.15) { //If axis 1 is off-center...
			liftPID.reset();//Reset the liftPID
			cubeLift.set(joy2.getRawAxis(1)/2);//Set the lift speed to the axis reading
			liftPID.setSetpoint(liftEnc.get());//Set the l
		} else {//If nothing is being pressed...
			liftPID.enable();
		}
		
		//TODO if stuttering is too much, switch to a setpoint change instead of manual override
		if(Math.abs(joy2.getRawAxis(5)) >= 0.15) { //If axis 5 is off-center...
			armPID.disable();
			arm.set(joy2.getRawAxis(5));//Set the arm motor to the axis reading
			armPID.setSetpoint(armEnc.get());//Set the armPID setpoint to the current encoder reading
		} else {//If the joystick isn't being touched...
			armPID.enable();
		}
		
		//TODO add piston behavior
		if(Math.abs(joy2.getRawAxis(2)) >= 0.25) { //If the left trigger is pulled...
			leftHolder.set(0.85); //Input cube
			rightHolder.set(0.85);
		} else if(Math.abs(joy2.getRawAxis(3)) >= 0.25) { //If right trigger is pulled...
			leftHolder.set(-0.25);//Soft spit
			rightHolder.set(-0.25);
		} else if(joy2.getRawButton(5)) { //If left bumper is pressed...
			leftHolder.set(-0.75); // Rotate cube
			rightHolder.set(0.75);
		} else if(joy2.getRawButton(6)) { //If right bumper is pressed...
			leftHolder.set(0.75); // Rotate cube
			rightHolder.set(-0.75);
		} else if(joy2.getRawButton(4)){ //If the Y button is pressed...
			leftHolder.set(-0.75);//Hard spit
			rightHolder.set(-0.75);
		} else {
			leftHolder.set(0);
			rightHolder.set(0);
		}
		
		read();//Read from sensors
	}
	
	void read() {//This puts data onto the smart dashboard
		SmartDashboard.putNumber("Lift encoder", liftEnc.get());
		SmartDashboard.putNumber("Lift PID", liftPID.get());
		SmartDashboard.putNumber("Lift speed", cubeLift.get());
		SmartDashboard.putNumber("Pressure", pressure.get());
		SmartDashboard.putNumber("Arm Encoder", armEnc.get());
		SmartDashboard.putNumber("Arm PID", armPID.get());
		SmartDashboard.putNumber("Arm speed", arm.get());
		SmartDashboard.putNumber("STRAIGHT PID", strPID.get());
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		
		SmartDashboard.putNumber("Axis 1", joy2.getRawAxis(1));
		
		SmartDashboard.putBoolean("Slot 1", slot1.get());
		SmartDashboard.putBoolean("Slot 2", slot2.get());
		SmartDashboard.putBoolean("Slot 3", slot3.get());
		SmartDashboard.putBoolean("Slot 4", (!slot1.get() && !slot2.get() && !slot3.get()));
		
		SmartDashboard.putString("results", matchInfo.getGameSpecificMessage());
	}
	
	@Override
	public void testPeriodic() {
	}
}
