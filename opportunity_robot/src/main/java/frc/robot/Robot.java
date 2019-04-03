/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

	
  private DifferentialDrive m_myRobot_frontMotors;
	private DifferentialDrive m_myRobot_backMotors;
	
	private Joystick xBox_controller;
	private Joystick joystick_controller;
	DigitalInput limitSwitch;
	DigitalInput haltClimbing;

	// create joystick stuff
	double leftJoyStick;
	double rightJoyStick;
	boolean flipDirection;
	int shouldIFlipDirection = 1;
	
	// create stuff for climbing arm
	double climbingArm_controllerUp;
	double climbingArm_controllerDown;
	private Talon climbingMotor = new Talon(8);
	double climbingArmSlowdownSpeed = 0.5;
	double climbingArmFinalSpeed;
	boolean climbingArmSafteyButton;
	
	// create stuff for ball shooter
	private Talon ballShootingMotor = new Talon(4);
	private Talon ballRaisingMotor = new Talon(7);
	double ballRaisingSpeed;
	double ballRaisingSpeedSlowdown = .6;
	
	double inSpeed = 0.5;
	double outSpeed = -0.4;
	double rocketHighSpeed = -.45;
	double rocketMiddleSpeed = -.50;
	double ballShooterCurrentSpeed = inSpeed;
	
	boolean inSpeedPressed;
	boolean outSpeedPressed;
	boolean rocketHighSpeedPressed;
	boolean rocketMiddleSpeedPressed;
	boolean ballShootingMotorButton;
	
	// create pneumatics stuff
	Compressor myCompressor = new Compressor(0);
	Solenoid ring_flipper_solenoid = new Solenoid(0);
	boolean trigger;
	boolean compressorOn;
	
	// create disk grabber stuff
	private Talon diskGrabberMotor = new Talon(5);
	int povValue;
	double diskArmOutSpeed = 0.5;
	double diskArmInSpeed = -0.5;
	double diskArmCurrentSpeed = 0.0;
	boolean diskArmIn;
	boolean diskArmOut;
	
	// create side value doubles
	double leftSideValue;
	double rightSideValue;
	
	// create booleans for speed control
	boolean speedUp;
	boolean speedDown;
	double maxSpeed = 1;
	double minSpeed = 0.65;
	double currentSpeed = maxSpeed;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Set Talon Assignments For Drive System
		// talons are the controllers for each motor
		Talon motor0 = new Talon(0);
		Talon motor1 = new Talon(1);
		Talon motor2 = new Talon(2);
		Talon motor3 = new Talon(3);
		
		// Create Drive Assignments
		m_myRobot_frontMotors = new DifferentialDrive(motor0, motor2); // front motors
		m_myRobot_backMotors = new DifferentialDrive(motor1, motor3); // back motors
		
		// Create Conection to xBox Controller
		xBox_controller = new Joystick(0);
		// Create Conection to joystick Controller
		joystick_controller = new Joystick(1);

		//// SAVE THIS ---- THIS IS THE WORKING CODE FOR ONE CAMERA
		// CameraServer.getInstance().startAutomaticCapture();


		myCompressor.stop();
		limitSwitch = new DigitalInput(0);
		haltClimbing = new DigitalInput(1);

		//// THIS IS THE NEW CAMERA CODE TO TEST
		////    LOOP CREATES MULTIPLE CAMERA SERVER INSTANCES
		////    AND NAMES THEM ramblerRobotics_Camera[int]
		// for (int i=0; i<2; i++) {
		// 	String cameraName = "ramblerRobotics_Camera" + i;
			new Thread(() -> {
				UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture(0);
				camera0.setResolution(320, 240);
				
				CvSink cvSink0 = CameraServer.getInstance().getVideo();
				CvSource outputStream0 = CameraServer.getInstance().putVideo("cameraName0", 320, 240);
				
				Mat source0 = new Mat();
				Mat output0 = new Mat();
				
				while(!Thread.interrupted()) {
					cvSink0.grabFrame(source0);
					Imgproc.cvtColor(source0, output0, Imgproc.COLOR_BGR2GRAY);
					outputStream0.putFrame(output0);
				}
			}).start();
			new Thread(() -> {
				UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(1);
				camera1.setResolution(320, 240);
				
				CvSink cvSink1 = CameraServer.getInstance().getVideo();
				CvSource outputStream1 = CameraServer.getInstance().putVideo("cameraName1", 320, 240);
				
				Mat source1 = new Mat();
				Mat output1 = new Mat();
				
				while(!Thread.interrupted()) {
					cvSink1.grabFrame(source1);
					Imgproc.cvtColor(source1, output1, Imgproc.COLOR_BGR2GRAY);
					outputStream1.putFrame(output1);
				}
			}).start();
		// }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
		robotMovement();

		// TRYING TO COMMENT THIS OUT TO MAKE AUTO_MODE WORK
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

		robotMovement();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  public void robotMovement() {
	  
		// do the lifting thing
		// boolean cb = xBox_controller.getRawButtonPressed(6);
		// if (cb) {

		// }

		flipDirection = xBox_controller.getRawButtonPressed(3);
		compressorOn = joystick_controller.getRawButton(8);
		
		if (compressorOn) {
			myCompressor.start();
		} else {
			myCompressor.stop();
		}
		
		if (flipDirection) {
			if (shouldIFlipDirection == -1) {
				shouldIFlipDirection = 1;
			} else if (shouldIFlipDirection == 1) {
				shouldIFlipDirection = -1;
			}
		}
		
		// get forward/backward joystick data
		leftJoyStick = xBox_controller.getRawAxis(1) * shouldIFlipDirection; // add this in to flip direction
		// get value of sideways joystick
		rightJoyStick = xBox_controller.getRawAxis(4);
		
		// get shooting buttons values
		inSpeedPressed = joystick_controller.getRawButtonPressed(3);
		outSpeedPressed = joystick_controller.getRawButtonPressed(4);
		rocketHighSpeedPressed = joystick_controller.getRawButtonPressed(6);
		rocketMiddleSpeedPressed = joystick_controller.getRawButtonPressed(5);
		ballShootingMotorButton = joystick_controller.getRawButton(2);
		
		// extend ring arm
//		povValue = joystick_controller.getPOV();
		diskArmIn = joystick_controller.getRawButton(11);
		diskArmOut = joystick_controller.getRawButton(12);
		
		// get trigger value
		trigger = joystick_controller.getTrigger();
		
		// climbing Arm
		climbingArmSafteyButton = xBox_controller.getRawButton(1);
		climbingArm_controllerUp = xBox_controller.getRawAxis(3);
		climbingArm_controllerDown = xBox_controller.getRawAxis(2) * -1;
		climbingArmFinalSpeed = (climbingArm_controllerUp + climbingArm_controllerDown) * climbingArmSlowdownSpeed;
    
		if ((climbingArmSafteyButton && !haltClimbing.get()) || (climbingArmSafteyButton && climbingArmFinalSpeed > 0)) {
		//if (climbingArmSafteyButton) {
			climbingMotor.set(climbingArmFinalSpeed);
		} else {
			climbingMotor.set(0);
		}
		
		
		//// ball shooter
		// raising /  lowering
		ballRaisingSpeed = joystick_controller.getRawAxis(1) * ballRaisingSpeedSlowdown * -1;
		
		if (!limitSwitch.get() || ballRaisingSpeed > 0) {
			ballRaisingMotor.set(ballRaisingSpeed);
		} else {
			ballRaisingMotor.set(0);
		}
		
		// shooter motor
		if (inSpeedPressed) { // set speed based on button pressed
			ballShooterCurrentSpeed = inSpeed;
		} else if (outSpeedPressed) {
			ballShooterCurrentSpeed = outSpeed;
		} else if (rocketMiddleSpeedPressed) {
			ballShooterCurrentSpeed = rocketMiddleSpeed;
		} else if (rocketHighSpeedPressed) {
			ballShooterCurrentSpeed = rocketHighSpeed;
		}
		
		if (ballShootingMotorButton) {
			ballShootingMotor.set(ballShooterCurrentSpeed);
		} else {
			ballShootingMotor.set(0);
		}
		
		// Disk Arm
//		if (povValue == 0) {
//			diskArmCurrentSpeed = diskArmOutSpeed;
//		} else if (povValue == 180) {
//			diskArmCurrentSpeed = diskArmInSpeed;
//		} else {
//			diskArmCurrentSpeed = 0.0;
//		}
		if (diskArmIn) {
			diskArmCurrentSpeed = 1;//diskArmInSpeed;
		} else if (diskArmOut) {
			diskArmCurrentSpeed = -1;// diskArmOutSpeed;
		} else {
			diskArmCurrentSpeed = 0;
		}
		diskGrabberMotor.set(diskArmCurrentSpeed);
		
		// set solenoid value
		if (trigger) {
			ring_flipper_solenoid.set(true);
		} else {
			ring_flipper_solenoid.set(false);
		}
		
		//// Speed Control
		speedUp = xBox_controller.getRawButtonPressed(4);
		speedDown = xBox_controller.getRawButtonPressed(2);
		
		// change speed of robot.
		if (speedUp) {
			currentSpeed = maxSpeed;
		} else if (speedDown) {
			currentSpeed = minSpeed;
		}
		
		leftSideValue = (leftJoyStick + rightJoyStick) * currentSpeed;
		rightSideValue = (leftJoyStick + (rightJoyStick * -1)) * currentSpeed;
		
		if (leftSideValue > 1) { leftSideValue = 1; }
		if (rightSideValue > 1) { rightSideValue = 1; }
		if (leftSideValue < -1) { leftSideValue = -1; }
		if (rightSideValue < -1) { rightSideValue = -1; }
		
		m_myRobot_frontMotors.tankDrive(leftSideValue * .75, rightSideValue);
		m_myRobot_backMotors.tankDrive(leftSideValue * .75, rightSideValue);
  }

}
