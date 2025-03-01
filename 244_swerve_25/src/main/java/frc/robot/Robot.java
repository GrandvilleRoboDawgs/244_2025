// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import java.io.ObjectInputFilter.Config;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.IterativeRobot;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private Command m_mauto2Command;
  private Command m_mauto4Command;
  private Command m_mauto3Command;
  private Command m_mauto5Command;
  private Command m_lbautoCommand;
  Timer time1 = new Timer();
    Command autoSelected;
  private final Spark m_ArmIntake;
  private final Spark m_AlgaeIntake;
  private final Spark m_AlgaePivot;
  private final Spark m_CoralLeft;
  private final Spark m_CoralRight;
  private Joystick operator = new Joystick(1);
  private final Joystick driver = new Joystick(0);
  private TalonFX intakepivot = new TalonFX(8);
  private SparkMax elevator;
  //private DutyCycleEncoder elevator_encoder = new DutyCycleEncoder(8);
  //private DutyCycleEncoder Arm_encoder = new DutyCycleEncoder(9);
  Encoder elevator_encoder = new Encoder(7,8, false, EncodingType.k2X);
  Encoder Arm_encoder = new Encoder(9, 6);
  


  public Robot() {
    m_robotContainer = new RobotContainer();
    m_AlgaeIntake = new Spark(0);
    m_ArmIntake = new Spark(4);
    m_AlgaePivot = new Spark(2);
    m_CoralLeft = new Spark(3);
    m_CoralRight = new Spark(1);
    elevator = new SparkMax(16, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    globalConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    elevator.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CameraServer.startAutomaticCapture();
    
  }
   
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //m_robotContainer = new RobotContainer();
    //m_lbautoCommand = m_robotContainer.m_lbautoCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putNumber("elevator_encoderValue", elevator_encoder.getDistance());
    SmartDashboard.putNumber("Arm_EncoderValue", Arm_encoder.get());
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();// Auto end
    
    }
  }

  @Override
  public void teleopPeriodic() {
    
    //Arm Intake
      if (driver.getRawButton(PS4Controller.Button.kR1.value)){
        m_ArmIntake.set(.75);


    } else if (driver.getRawButton(PS4Controller.Button.kL1.value)) {
        m_ArmIntake.set(-.75);

    } else {
      m_ArmIntake.stopMotor();

    } 
    //Algae Intake
      if (operator.getRawButton(PS4Controller.Button.kL3.value)){
       m_AlgaeIntake.set(.5);

    } else if (operator.getRawButton(PS4Controller.Button.kR3.value)){
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


    // Coral Left/Right
    if (operator.getRawButton(PS4Controller.Button.kR2.value)){
      m_CoralLeft.set(.5);
      m_CoralRight.set(.5);

    } else if (operator.getRawButton(PS4Controller.Button.kL2.value)){
      m_CoralLeft.set(-.5);
      m_CoralRight.set(-.5);

    } else {
      m_CoralLeft.stopMotor();
      m_CoralRight.stopMotor();    
    }
   
    
    // IntakePivot*
    if (driver.getRawAxis(PS4Controller.Axis.kL2.value) > .1) {
      intakepivot.set(driver.getRawAxis(PS4Controller.Axis.kL2.value) * .3);
    } else if (driver.getRawAxis(PS4Controller.Axis.kR2.value) > .1) {
      intakepivot.set(-driver.getRawAxis(PS4Controller.Axis.kR2.value) * .3);
    } else {
      intakepivot.set(0.0);
    }

    //elevator
  
    
      if (operator.getRawButton(PS4Controller.Button.kTriangle.value)) {
          if (elevator_encoder.get() < 19000 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 19001 && elevator_encoder.get() < 19500) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 20080) {
            elevator.set(0.0);
        }
      }
        else if (operator.getRawButton(PS4Controller.Button.kSquare.value)) {
          if (elevator_encoder.get() < 9000 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 9001 && elevator_encoder.get() < 9500) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 9500) {
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
          if (elevator_encoder.get() < 2800 ) {
            elevator.set(.85);
        } else if (elevator_encoder.get() > 2801 && elevator_encoder.get() <3300 ) {
            elevator.set(.5);
        } else if (elevator_encoder.get() > 3300) {
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
