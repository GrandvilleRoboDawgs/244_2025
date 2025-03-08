// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

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
//import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.math.geometry.Rotation2d;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  private final Spark m_ArmIntake;
  private final Spark m_AlgaeIntake;
  private final Spark m_AlgaePivot;
  private final Spark m_CoralLeft;
  private final Spark m_CoralRight;
  private Joystick operator = new Joystick(1);
  private final Joystick driver = new Joystick(0);
  private TalonFX intakepivot = new TalonFX(8);
  private SparkMax elevator;
  private SparkMax Coral;
  //private DutyCycleEncoder elevator_encoder = new DutyCycleEncoder(8);
  //private DutyCycleEncoder Arm_encoder = new DutyCycleEncoder(9);
  Encoder elevator_encoder = new Encoder(7,8, false, EncodingType.k2X);
  Encoder Arm_encoder = new Encoder(9, 6);
  private DigitalInput LimitSwitch;


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
    intakepivot.setNeutralMode(NeutralModeValue.Brake);
    //m_AlgaePivot.IdleMode(IdleMode.kBrake);
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

    

    // if(timer.get() < 1.0) {
      // frontLeft.set(-.5);
      // backRight.set(-.5);
      // frontRight.set(-.5);
    //   m_robotContainer.driveBack();
    //   // m_robotContainer.drivetrain.setControl(drive.withVelocityX(-.5 * 4.73));
    //   // m_robotContainer.drivetrain.sysIdDynamic(Direction.kReverse);
    //   // m_robotContainer.drivetrain.getModule(0).getDriveMotor().set(.5);
    //   // m_robotContainer.drivetrain.setControl(drive.withVelocityX(-.5* 4.73));
    //   // m_robotContainer.drivetrain.applyRequest(() -> drive.withVelocityX(-.5 * 4.73));
    //   // m_robotContainer.drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-.5, 0)));
      if (timer.get() >= 1.2 && timer.get() < 1.5) {
      intakepivot.set(.15);
    } else if(timer.get() >= 1.5 && timer.get() < 3.0) {
      intakepivot.set(0);
      m_ArmIntake.set(-.6);
    } else {
      m_ArmIntake.set(0);
      intakepivot.set(0);
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();// Auto end
    
    // }
  }

  @Override
  public void teleopPeriodic() {
    
    //Arm Intake
      if (driver.getRawButton(PS4Controller.Button.kR2.value)){
        m_ArmIntake.set(.75);


    } else if (driver.getRawButton(PS4Controller.Button.kL2.value)) {
        m_ArmIntake.set(-.75);

    } else {
      m_ArmIntake.stopMotor();

    } 
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
    // if (!LimitSwitch.get()) {
    //     Coral.set(.1);
  
    // Coral Left/Right
    //} else
     if (operator.getRawButton(PS4Controller.Button.kR2.value)){
      //Coral.set(.65);
      m_CoralLeft.set(.5);
      m_CoralRight.set(.5);

    } else if (operator.getRawButton(PS4Controller.Button.kL2.value)){
      // Coral.set(-.5);
      m_CoralLeft.set(-.5);
      m_CoralRight.set(-.5);

    } else {
      //Coral.set(0);
      m_CoralLeft.stopMotor();
      m_CoralRight.stopMotor();    
    }
   
    
    // IntakePivot*
      if (operator.getRawButton(PS4Controller.Button.kL3.value)) {
        intakepivot.set(.5);
    } else if (operator.getRawButton(PS4Controller.Button.kR3.value)) {
        intakepivot.set(-.5);
    } else {
        intakepivot.set(0.0);
    }

    //elevator
  
    
      if (operator.getRawButton(PS4Controller.Button.kTriangle.value)) {
          if (elevator_encoder.get() < 18050 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 18100 && elevator_encoder.get() < 18650) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 18700) {
            elevator.set(0.0);
        }
      }
        else if (operator.getRawButton(PS4Controller.Button.kSquare.value)) {
          if (elevator_encoder.get() < 8353 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 8353 && elevator_encoder.get() < 8853) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 8853) {
            elevator.set(0.0);
        } 

      } else if (operator.getRawButton(PS4Controller.Button.kCross.value)) {
          if (elevator_encoder.get() < .31 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > .33 ) {
            elevator.set(-.75);
        } else if (elevator_encoder.get() > .311 && elevator_encoder.get() < .329) {
            elevator.set(0.0);
        } else {
          
        }

      } else if (operator.getRawButton(PS4Controller.Button.kCircle.value)) {
          if (elevator_encoder.get() < 2300 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 2300 && elevator_encoder.get() <2695 ) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 2695) {
            elevator.set(0.0);
        } else {
          
        }
      
      } else if (elevator_encoder.get() <= 0) {
        elevator.set(0.0);

      } else if (operator.getRawButton(PS4Controller.Button.kShare.value)) {
        elevator.set(0.75);

      } else if (operator.getRawButton(PS4Controller.Button.kOptions.value)) {
        elevator.set(-.75);

      } else {
        elevator.set(0);

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
