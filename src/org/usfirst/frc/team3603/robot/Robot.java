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
	MyEncoder driveEnc = new MyEncoder(left1, true, lowGear);//TODO test, talon
	
	PressureSensor pressure = new PressureSensor(0); //Pressure sensor
	AHRS gyro = new AHRS(Port.kMXP); //NavX
	
	PIDController strPID = new PIDController(0.15, 0, 0, gyro, new Spark(7)); //PID controller for driving straight
	PIDController liftPID = new PIDController(0.001, 0, 0, liftEnc, cubeLift); //PID controller for lift
	PIDController armPID = new PIDController(0.08, 0, 0, armEnc, arm); //PID controller for arm
	
	DigitalInput slot1 = new DigitalInput(2); //Digital inputs for the auton switch
	DigitalInput slot2 = new DigitalInput(3);
	DigitalInput slot3 = new DigitalInput(4);
	DigitalInput highSwitch = new DigitalInput(5);
	DigitalInput lowSwitch = new DigitalInput(6);
	
	DriverStation matchInfo = DriverStation.getInstance(); //Object to get switch/scale colors

	AutonType autonMode; //Enumerator for the autonomous mode
	int step; //The auton step
	final static double scaleStartHeight = -15000;//Double for scale encoder position
	final static double switchHeight = -12000;//Double for the switch encoder position
	final static double scaleFinishHeight = -24000;
	final static double lowGear = Math.PI*4/30680;//TODO check these and scale with wheel circumference
	final static double highGear= lowGear; 
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
		driveEnc.reset();
	}
	
	@Override
	public void disabledPeriodic() {
		read();
	}
	
	@Override
	public void autonomousInit() {
		String sides = matchInfo.getGameSpecificMessage(); //Get the switch and scale colors
		liftPID.setOutputRange(-0.5, 0.5);
		int position;
		if(slot1.get()) { //Logic to find the auton rotating switch position
			position = 1;
		} else if(slot2.get()) {
			position = 2;
		} else if(slot3.get()) {
			position = 3;
		} else {
			position = 4;
		}
		
		String LRL = "LRL";
		String RLR = "RLR";
		String LLL = "LLL";
		String RRR = "RRR";
		
		System.out.println("Position: " + position);
		System.out.println("Sides: " + sides);
		if(position == 1) { //If we are in position 1...
			if(sides.equals(LLL)) {//If the switch and scale is on our side...
				autonMode = AutonType.leftSwitch;//Set the auton mode to the left side of the switch
				System.out.println("Autonomous mode: left scale");
			} else if(sides.equals(RRR)) {//If neither the switch or scale are on our side...
				autonMode = AutonType.straight; //Cross the auto line
				System.out.println("Autonomous mode: straight");
			} else if(sides.equals(LRL)) {//If only the switch is on our side...
				autonMode = AutonType.leftSwitch;//Set the auton mode to the left side of the switch
				System.out.println("Autonomous mode: left switch");
			} else if(sides.equals(RLR)) {//If only the scale is on our side...
				autonMode = AutonType.leftScale;//Set the auton mode to the left scale
				System.out.println("Autonomous mode: left scale");
			}
		} else if(position == 2) { //If we are in position 2...
			if(sides.equals(LLL) || sides.equals(LRL)) {//If the switch is on the left...
				autonMode = AutonType.leftMiddle;//Set the auton mode to left middle
				System.out.println("Autonomous mode: middle left");
			} else if(sides.equals(RLR) || sides.equals(RRR)) {//If the switch is on the right side...
				autonMode = AutonType.rightMiddle;//Set the auton mode to right middle
				System.out.println("Autonomous mode: middle right");
			}
		} else if(position == 3) {//If we are in position 3...
			if(sides.equals(LLL)) {//If neither the switch or scale is on our side...
				autonMode = AutonType.straight;//Set the auton mode to straight
				System.out.println("Autonomous mode: straight");
			} else if(sides.equals(RRR)) {//If both the switch and the scale are on our side...
				autonMode = AutonType.rightSwitch;//Do the switch
				System.out.println("Autonomous mode: right scale");
			} else if(sides.equals(LRL)) {//If only the scale is on our side...
				autonMode = AutonType.rightScale;//Do the scale
				System.out.println("Autonomous mode: right scale");
			} else if(sides.equals(RLR)) {//If only the switch is on our side...
				autonMode = AutonType.rightSwitch;//Do the switch
				System.out.println("Autonomous mode: right switch");
			}
		} else if(position == 4) {//If the auton switch is in position 4....
			autonMode = AutonType.pos3LeftScale;//Override and drive straight
			System.out.println("Autonomous mode: Override straight");
		}
		if(autonMode == null) {
			autonMode = AutonType.straight;
			System.out.println("ERROR: autonMode is null. Defaulting to cross auto line.");
		}
		
		strPID.setSetpoint(0); //Set the setpoint of the drive straight PID to 0 degrees
		armPID.setSetpoint(75);
		gyro.reset(); //Set the gyro angle to 0
		driveEnc.reset(); //Set the touchless encoder to 0
		armEnc.reset();
		armEnc.reset();//Reset the arm encoder
		strPID.enable();
		shift.set(out);
		step = 0; //set the auton step to step 0
	}//TODO add pneumatics
	
	@Override
	public void autonomousPeriodic() {
		release.set(0.5);//Release the arm by raising the servo
		
		read();//Read from sensors and put the info on the smart dashboard
		switch(autonMode) {
		case straight:
			straight(); //Drive straight for auton
			break;
		case leftSwitch:
			leftSwitch(); //Go to the left side of the switch 
			break;
		case rightSwitch:
			rightSwitch(); //Go to the right side of the switch
			break;
		case leftScale:
			leftScale(); //Go to the left side of the scale
			break;
		case rightScale:
			rightScale(); //Go to the right side of the scale
			break;
		case rightMiddle:
			rightMiddle(); //Go to the front right side of the switch
			break;
		case leftMiddle:
			leftMiddle(); //Go to the front left side of the switch
			break;
		case pos3LeftScale:
			pos3LeftScale();
			break;
		}
	}
	
	void pos3LeftScale() {
		switch(step) {
		case 0:
			step = 1;
			break;
		case 1:
			if(driveEnc.get() < 215) {
				mainDrive.arcadeDrive(0.75, strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
				strPID.setSetpoint(-90);
				step = 2;
			}
			break;
		case 2:
			if(gyro.getAngle() > -90) {
				mainDrive.arcadeDrive(0, -0.4);
			} else {
				liftPID.setSetpoint(scaleStartHeight);
				liftPID.enable();
				armPID.enable();
				mainDrive.arcadeDrive(0, 0);
				strPID.setSetpoint(-90);
				driveEnc.reset();
				step = 3;
			}
			break;
		case 3:
			if(driveEnc.get() < 168) {
				mainDrive.arcadeDrive(0.75, strPID.get());
			} else {
				liftPID.setSetpoint(scaleFinishHeight);
				mainDrive.arcadeDrive(0, 0);
				step = 4;
			}
			break;
		case 4:
			if(gyro.getAngle() < 0) {
				mainDrive.arcadeDrive(0, 0.4);
			} else {
				mainDrive.arcadeDrive(0, 0);
				strPID.setSetpoint(0);
				driveEnc.reset();
				step = 5;
			}
			break;
		case 5:
			if(driveEnc.get() < 36) {
				mainDrive.arcadeDrive(0.3, strPID.get());
			} else {
				mainDrive.arcadeDrive(0, 0);
			}
			break;
		case 6:
			if(liftEnc.get() > -23000) {
			} else {
				leftHolder.set(-0.5);
				rightHolder.set(-0.5);
			}
			break;
		}
	}
	
	@Override
	public void teleopInit() {
		armPID.setSetpoint(armEnc.get());
		liftPID.setSetpoint(liftEnc.get());
		liftPID.setOutputRange(-0.7, 0.7);
		liftPID.disable();
		shift.set(in);
		strPID.disable();
    	driveEnc.reset();
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
		double rot = Math.pow(joy1.getRawAxis(2), 1)/1.25; //Double to store the joystick's x axis
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
		
		if(joy1.getRawButtonReleased(12)) {
			driveEnc.reset();
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
		
		/**
		if(!highSwitch.get()) {
			liftPID.disable();
			if(joy2.getRawAxis(1) <= -0.15) {
				cubeLift.set(0);
			} else {
				cubeLift.set(joy2.getRawAxis(1));
				liftPID.setSetpoint(liftEnc.get());
			}
		} else if(!lowSwitch.get()) {
			liftPID.disable();
			if(joy2.getRawAxis(1) >= 0.15) {
				cubeLift.set(0);
			} else {
				cubeLift.set(joy2.getRawAxis(1));
				liftPID.setSetpoint(liftEnc.get());
			}
		} else if(Math.abs(joy2.getRawAxis(1)) >= 0.15) {
			liftPID.disable();
			cubeLift.set(joy2.getRawAxis(1));
			liftPID.setSetpoint(liftEnc.get());
		} else {
			liftPID.enable();
		}
		*/
		
		if(Math.abs(joy2.getRawAxis(1)) >= 0.15) { //If axis 1 is off-center...
			liftPID.reset();//Reset the liftPID
			cubeLift.set(joy2.getRawAxis(1)*3/4);//Set the lift speed to the axis reading
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
			leftHolder.set(-0.35);//Soft spit
			rightHolder.set(-0.35);
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
		SmartDashboard.putNumber("Drive distance", driveEnc.get());
		
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
	
	enum AutonType {//A list of different auton types
		rightScale, leftScale, rightSwitch, leftSwitch, straight, rightMiddle, leftMiddle, pos3LeftScale
	}
	
	void leftMiddle() {
		shift.set(in);
		switch(step) {
		case 0:
			liftPID.setSetpoint(switchHeight);//Set the liftPID to 12000
			step = 1;
			break;
		case 1://Step one of the left auton
			if(driveEnc.get() < 10) {//If the drive distance is less than 10 inches...
				mainDrive.arcadeDrive(0.75, strPID.get());//Drive at -3/4 speed and use the PID
			} else {//If the drive distance greater than 10 inches...
				mainDrive.arcadeDrive(0, 0);///Stop
				strPID.disable();
				liftPID.enable();
				step = 2;//Set the step to 2
			}
			break;
		case 2://Step 2 of the left middle auton
			if(gyro.getAngle() > -60) {//If the current angle is greater than -60
				mainDrive.arcadeDrive(0, -0.4);//Slowly turn left
			} else { //If it has reached the -60 degree mark...
				mainDrive.arcadeDrive(0, 0);//Stop
				strPID.setSetpoint(-60);//Set the straightPID setpoint to -60
				strPID.enable();//Enable the straight PID
				driveEnc.reset();//Reset the touchless encoder
				armPID.enable();
				step = 3;//Set the step to 3
			}
			break;
		case 3://Step 3 of the left middle auton
			if(driveEnc.get() < 144) {//If the robot has driven less than 144 inches...
				mainDrive.arcadeDrive(0.75, strPID.get());//Drive at -3/4 and drive straight with PID
			} else {//If the robot has driven more than 144 inches...
				mainDrive.arcadeDrive(0, 0);//Stop
				strPID.disable();
				step = 4;//Set the step to 4
			}
			break;
		case 4://Step 4
			if(gyro.getAngle() < 0) {//If the current gyro angle is less than 0...
				mainDrive.arcadeDrive(0, 0.4);//Slowly turn right
			} else {//If it has completely turned 60 degrees...
				mainDrive.arcadeDrive(0, 0);//Stop
				step = 5;//Set the step to 5
			}
			break;
		case 5://Step 5
			leftHolder.set(-0.5);//Output the cube into the switch
			rightHolder.set(-0.5);
			break;
		}
	}
	
	void rightMiddle() {//Method to do the right side of the switch when in position 2
		switch(step) {
		case 0:
			liftPID.setSetpoint(switchHeight);//set the lift PID setpoint to 10000
			armPID.enable();//Enable the armPID
			liftPID.enable();//Enable the liftPID
			step = 1;
			break;
		case 1:
			if(driveEnc.get() < 83) {//If the robot has driven less than 83 inches...
				mainDrive.arcadeDrive(0.75, strPID.get());//Drive straight at 3/4 speed
			} else {//If it has driven 83 inches...
				mainDrive.arcadeDrive(0, 0);//Stop
				step = 2;//Set the step to 2
			}
			break;
		case 2:
			if(liftEnc.get() > -10000) {
			} else {
				leftHolder.set(-0.3);//Output the cube
				rightHolder.set(-0.3);
				strPID.disable();
			}
			break;
		}
	}
	
	void straight() {//Auton the drive straight
		if(driveEnc.get() < 83) {//TODO Check /If the robot has driven less than 83 inches...
			mainDrive.arcadeDrive(0.75, strPID.get());//Drive straight at -3/4 speed
		} else {//Else
			mainDrive.arcadeDrive(0, 0);//Stop
		}
	}
	
	void rightScale() {//Auton for the right side of the scale
		switch(step) {
		case 0:
			liftPID.setSetpoint(scaleStartHeight);//Set the lift setpoint to 15000
			liftPID.enable();//Enable the liftPID
			armPID.enable();
			step = 1;
			break;
		case 1:
			if(driveEnc.get() < 234) {//If the robot has driven less than 252 inches
				mainDrive.arcadeDrive(0.75, strPID.get()); //Drive forwards at 3/4 speed and drive straight
			} else {//If the robot has driven further than 252 inches
				mainDrive.arcadeDrive(0, 0);//Stop
				step = 2;//Set the step to 2
				strPID.disable();
				liftPID.setSetpoint(scaleFinishHeight);//TODO check this /Set the lift setpoint to max height
			}
			break;
		case 2://Step 2
			if(gyro.getAngle() > -45) {//If the current gyro angle is greater than -35 degrees
				mainDrive.arcadeDrive(0, -0.4);//Turn left
			} else {//Else
				step = 3;//Set the step to 3
				mainDrive.arcadeDrive(0, 0);//Stop
			}
			break;
		case 3://Step 3 TODO change to limit switch
			if(liftEnc.get() > -23000) {//Wait until the lift is up
			} else {
				leftHolder.set(-0.75);
				rightHolder.set(-0.75);
			}
			break;
		}
	}
	
	void leftScale() {//Auton for the left side of the scale
		switch(step) {
		case 0:
			liftPID.setSetpoint(scaleStartHeight);//Set the lift setpoint to a little over half way
			armPID.enable();//Enable the arm PID
			liftPID.enable();//Enable the lift PID
			step = 1;
			break;
		case 1:
			if(driveEnc.get() < 234) {//If the robot has driven less than 252 inches...
				mainDrive.arcadeDrive(0.75, strPID.get());//Drive straight
			} else {//Else
				mainDrive.arcadeDrive(0, 0);//Stop
				step = 2;//go to step 2
				strPID.disable();
				liftPID.setSetpoint(scaleFinishHeight);//TODO check this /Set the lift pid setpoint to max
			}
			break;
		case 2://Step 2
			if(gyro.getAngle() < 45) {//If the current gyro angle is less than 45 degrees
				mainDrive.arcadeDrive(0, 0.4);//Turn right
			} else {
				step = 3;//go to step 3
				driveEnc.reset();//reset the touchless encoder
				mainDrive.arcadeDrive(0, 0);//stop
			}
			break;
		case 3:
			if(liftEnc.get() > -23000) {//dont output the cube until the lift is up
			} else {
				leftHolder.set(-0.75);
				rightHolder.set(-0.75);
			}
			break;
		}
	}
	
	void leftSwitch() {//auton for the left side of the switch
		switch(step) {
		case 0:
			liftPID.setSetpoint(switchHeight);//St the lift to 10000
			liftPID.enable();//enable the lift pid
			step = 1;
			break;
		case 1:
			if(driveEnc.get() < 131) {//if the robot has driven less than 149 inches...
				mainDrive.arcadeDrive(0.75, strPID.get());//drive straight
			} else {
				mainDrive.arcadeDrive(0, 0);//stop
				strPID.disable();
				armPID.enable();
				step = 2;//go to step 2
			}
			break;
		case 2://step 2
			if(gyro.getAngle() < 80) {//if the current angle is less than 90 degrees
				mainDrive.arcadeDrive(0, 0.6);//turn right
			} else {//else
				step = 3;//go to step 3
				driveEnc.reset();//reset the touchless encoder
				mainDrive.arcadeDrive(0, 0);//stop
			}
			break;
		case 3:
			if(driveEnc.get() < 9) {//if the robot has driven less than 9 inches
				mainDrive.arcadeDrive(0.5, 0);//drive forwards
			} else {//else
				step = 4;//go to step 4
				mainDrive.arcadeDrive(0, 0);//stop
			}
			break;
		case 4:
			cubeLift.set(-liftPID.get());//keep the lift in place
			leftHolder.set(-0.35);//shoot the cube
			rightHolder.set(-0.35);
			break;
		}
	}
	
	void rightSwitch() {//auton for the right side of the switch
		switch(step) {
		case 0:
			liftPID.setSetpoint(switchHeight);//set the lift setpoint
			liftPID.enable();//enable lift PID
			step = 1;
			break;
		case 1://step 1
			if(driveEnc.get() < 131) {//if the robot has driven less tham 149 inches...
				mainDrive.arcadeDrive(0.75, strPID.get());//drive straight
			} else {//else
				mainDrive.arcadeDrive(0, 0);//stop
				strPID.disable();
				armPID.enable();
				step = 2;//go to step 2
			}
			break;
		case 2://step 2
			if(gyro.getAngle() > -80) {//turn left until -90 degrees
				mainDrive.arcadeDrive(0, -0.6);
			} else {
				step = 3;//go to step 3
				driveEnc.reset();//reset the encoder
				mainDrive.arcadeDrive(0, 0);//stop
			}
			break;
		case 3:
			if(driveEnc.get() < 9) {//drive forward 9 inches
				mainDrive.arcadeDrive(0.5, 0);
			} else {
				step = 4;//go to step 4
				mainDrive.arcadeDrive(0, 0);//stop
			}
			break;
		case 4:
			leftHolder.set(-0.35);//output the cube
			rightHolder.set(-0.35);
			break;
		}
	}
}
