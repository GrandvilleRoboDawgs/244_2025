// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import java.lang.Thread.State;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix6.hardware.Pigeon2;
//import static edu.wpi.first.units.Units.Percent;
//import java.io.ObjectInputFilter.Config;
//import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.REVLibError;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.math.geometry.Rotation2d;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private Thread m_visionThread;
  private final Spark m_ArmIntake;
  private final Spark m_AlgaeIntake;
  private final Spark m_AlgaePivot;
  private final Spark m_CoralLeft;
  private final Spark m_CoralRight;
  private Servo m_Servo;
  private Joystick operator = new Joystick(1);
  private final Joystick driver = new Joystick(0);
  private TalonFX winch = new TalonFX(8);
  private SparkMax elevator;
  private SparkMax Coral;
  //private DutyCycleEncoder elevator_encoder = new DutyCycleEncoder(8);
  //private DutyCycleEncoder Arm_encoder = new DutyCycleEncoder(9);
  Encoder elevator_encoder = new Encoder(7,8, false, EncodingType.k2X);
  Encoder Arm_encoder = new Encoder(9, 6);
  private DigitalInput LimitSwitch;
  private final Pigeon2 pigeon = new Pigeon2(15, "rio");


  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Timer timer;
  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //           .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_AlgaeIntake = new Spark(0);
    m_ArmIntake = new Spark(4);
    m_AlgaePivot = new Spark(2);
    m_CoralLeft = new Spark(3);
    m_CoralRight = new Spark(1);
    elevator = new SparkMax(16, MotorType.kBrushless);
    Coral = new SparkMax(17, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    globalConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    elevator.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CameraServer.startAutomaticCapture();
    timer = new Timer();
    LimitSwitch = new DigitalInput(5);
    m_Servo = new Servo(5);
    double tx = LimelightHelpers.getTX("limelight");

// //Camera stuff
//     m_visionThread =
//         new Thread(
//             () -> {
//               // Get the UsbCamera from CameraServer
//               UsbCamera camera = CameraServer.startAutomaticCapture();
//               // Set the resolution
//               camera.setResolution(640, 480);

//               // Get a CvSink. This will capture Mats from the camera
//               CvSink cvSink = CameraServer.getVideo();
//               // Setup a CvSource. This will send images back to the Dashboard
//               CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

//               // Mats are very memory expensive. Lets reuse this Mat.
//               Mat mat = new Mat();

//               // This cannot be 'true'. The program will never exit if it is. This
//               // lets the robot stop this thread when restarting robot code or
//               // deploying.
//               while (!Thread.interrupted()) {
//                 // Tell the CvSink to grab a frame from the camera and put it
//                 // in the source mat.  If there is an error notify the output.
//                 if (cvSink.grabFrame(mat) == 0) {
//                   // Send the output the error.
//                   outputStream.notifyError(cvSink.getError());
//                   // skip the rest of the current iteration
//                   continue;
//                 }
//                 // Put a rectangle on the image
//                 Imgproc.rectangle(
//                     mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
//                 // Give the output stream a new image to display
//                 outputStream.putFrame(mat);
//               }
//             });
//     m_visionThread.setDaemon(true);
//     m_visionThread.start();




  }
   
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();
    //m_lbautoCommand = m_robotContainer.m_lbautoCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putNumber("elevator_encoderValue", elevator_encoder.getDistance());
    SmartDashboard.putNumber("Arm_EncoderValue", Arm_encoder.get());
    SmartDashboard.putBoolean("limitswitch", LimitSwitch.get());
    SmartDashboard.putNumber("x value", LimelightHelpers.getTX("limelight"));
    winch.setNeutralMode(NeutralModeValue.Brake);
    // m_AlgaePivot.IdleMode(IdleMode.kBrake);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
 
}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    timer.start();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {



    //   if (timer.get() >= 1.2 && timer.get() < 1.5) {
    //   intakepivot.set(.25);
    // } else if(timer.get() >= 1.5 && timer.get() < 3.0) {
    //   intakepivot.set(0);
    //   m_ArmIntake.set(-.6);
    // } else {
    //   m_ArmIntake.set(0);
    //   intakepivot.set(0);
    // }



    if (timer.get() >= 1 && timer.get() < 3) {
        if (elevator_encoder.get() < 19050 ) {
          elevator.set(.85);
      } else if (elevator_encoder.get() > 19050 && elevator_encoder.get() < 19550) {
          elevator.set(.5);
      } else if (elevator_encoder.get() > 19550) {
          elevator.set(0.019);
      }
      // <3.6 for side, <8.6 for middle
    } else if (timer.get() >= 3.3 && timer.get() < 8.6) {
        m_CoralLeft.set(.44);
        m_CoralRight.set(.44);
    } 

    //for middle auto
      else if (timer.get() >= 9 && timer.get() < 15) {
        if (elevator_encoder.get() < .31 ) {
          elevator.set(.25);
      } else if (elevator_encoder.get() > .53 ) {
          elevator.set(-.8);
      } else if (elevator_encoder.get() > .451 && elevator_encoder.get() < .529) {
          elevator.set(-.40);
      } else if (elevator_encoder.get() > .311 && elevator_encoder.get() < .45) {
          elevator.set(0.019);
      }
    }
    
    //for side auto
    // else if (timer.get() >= 3.9 && timer.get() < 5) {
    //     if (elevator_encoder.get() < .31 ) {
    //       elevator.set(.25);
    //   } else if (elevator_encoder.get() > .53 ) {
    //       elevator.set(-.8);
    //   } else if (elevator_encoder.get() > .451 && elevator_encoder.get() < .529) {
    //       elevator.set(-.40);
    //   } else if (elevator_encoder.get() > .311 && elevator_encoder.get() < .45) {
    //       elevator.set(0.019);
    //   }
    // } else if (timer.get() >= 4 && timer.get() < 10) {
    //     if (!LimitSwitch.get()) {
    //       m_CoralLeft.set(.44);
    //       m_CoralRight.set(.44);
    //   } else {
    //       m_CoralLeft.set(0);
    //       m_CoralRight.set(0);
    //   }
    // } else if (timer.get() >= 11 && timer.get() < 13) {
    //     if (elevator_encoder.get() < 19050 ) {
    //       elevator.set(.85);
    //   } else if (elevator_encoder.get() > 19050 && elevator_encoder.get() < 19550) {
    //       elevator.set(.5);
    //   } else if (elevator_encoder.get() > 19550) {
    //       elevator.set(0.019);
    //   }
    // } else if (timer.get() >= 13 && timer.get() < 13.3) {
    //     m_CoralLeft.set(.5);
    //     m_CoralRight.set(.5);
    // } else if (timer.get() >= 14 && timer.get() < 15) {
    //     if (elevator_encoder.get() < .31 ) {
    //       elevator.set(.25);
    //   } else if (elevator_encoder.get() > .53 ) {
    //       elevator.set(-.80);
    //   } else if (elevator_encoder.get() > .451 && elevator_encoder.get() < .529) {
    //       elevator.set(-.40);
    //   } else if (elevator_encoder.get() > .311 && elevator_encoder.get() < .45) {
    //       elevator.set(0.019);
    //   }
    // }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    
    // }
  }

  @Override
  public void teleopPeriodic() {
    
    // //Arm Intake
    //   if (driver.getRawButton(PS4Controller.Button.kR2.value)){
    //     m_ArmIntake.set(.75);


    // } else if (driver.getRawButton(PS4Controller.Button.kL2.value)) {
    //     m_ArmIntake.set(-.75);

    // } else {
    //   m_ArmIntake.set(0);

    // } 

    //Algae Intake
      if (driver.getRawButton(PS4Controller.Button.kL1.value)){
       m_AlgaeIntake.set(.5);

    } else if (driver.getRawButton(PS4Controller.Button.kR1.value)){
      m_AlgaeIntake.set(-.5);

    } else {
      m_AlgaeIntake.stopMotor();

    }
    //Algae pivot
      if (operator.getRawButton(PS4Controller.Button.kL1.value)){
      m_AlgaePivot.set(.45);

    } else if (operator.getRawButton(PS4Controller.Button.kR1.value)){
      m_AlgaePivot.set(-.45);
    
    } else {
      m_AlgaePivot.set(0);
    }
    
    // Coral limit
    if (operator.getRawButton(PS4Controller.Button.kL2.value) && !LimitSwitch.get()) {
     m_CoralLeft.set(.44);
     m_CoralRight.set(.44);
  
    //Coral Left/Right
    } else if (operator.getRawButton(PS4Controller.Button.kR2.value)){
      //Coral.set(.65);
      m_CoralLeft.set(.44);
      m_CoralRight.set(.44);

    } else if (operator.getRawButton(PS4Controller.Button.kTouchpad.value)){
      // Coral.set(-.5);
      m_CoralLeft.set(-.5);
      m_CoralRight.set(-.5);

    } else {
      //Coral.set(0);
      m_CoralLeft.stopMotor();
      m_CoralRight.stopMotor();    
    }
   
    // ramp servo
      if (operator.getRawButton(PS4Controller.Button.kL3.value)) {
        m_Servo.set(1);
    } else if (operator.getRawButton(PS4Controller.Button.kR3.value)) {
        m_Servo.set(0);
    } else {

    }
    
    // winch
      if (driver.getRawButton(PS4Controller.Button.kL2.value)) {
        winch.set(.85);
    } else if (driver.getRawButton(PS4Controller.Button.kR2.value)) {
        winch.set(-.85);
    } else {
        winch.set(0.0);
    }

    //elevator
  
    
        if (operator.getRawButton(PS4Controller.Button.kTriangle.value)) {
          if (elevator_encoder.get() < 19050 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 19050 && elevator_encoder.get() < 19550) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 19550) {
            elevator.set(0.019);
        }
      } else if (operator.getRawButton(PS4Controller.Button.kSquare.value)) {
          if (elevator_encoder.get() < 8353 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 8353 && elevator_encoder.get() < 8853) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 8853) {
            elevator.set(0.019);
        } 

      } else if (operator.getRawButton(PS4Controller.Button.kCross.value)) {
          if (elevator_encoder.get() < .31 ) {
            elevator.set(.25);
        } else if (elevator_encoder.get() > .53 ) {
            elevator.set(-.80);
        } else if (elevator_encoder.get() > .451 && elevator_encoder.get() < .529) {
            elevator.set(-.40);
        } else if (elevator_encoder.get() > .311 && elevator_encoder.get() < .45) {
            elevator.set(0.019);
        }
    

      } else if (operator.getRawButton(PS4Controller.Button.kCircle.value)) {
          if (elevator_encoder.get() < 2300 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 2300 && elevator_encoder.get() <2695 ) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 2695) {
            elevator.set(0.019);
        } else {
          
        }

      } else if (operator.getRawButton(PS4Controller.Button.kShare.value)) {
        elevator.set(0.25);

      } else if (operator.getRawButton(PS4Controller.Button.kOptions.value)) {
        elevator.set(-.85);

      } else {
        elevator.set(.019);

      }
      
    
    }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
