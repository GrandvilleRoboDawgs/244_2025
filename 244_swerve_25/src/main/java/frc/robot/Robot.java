// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;


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
  private TalonFX intakepivot = new TalonFX(9);
  private final SparkMax elevator = new SparkMax(16, MotorType.kBrushless);
  //public final SparkMaxConfigAccessor configAccessor;

  

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_AlgaeIntake = new Spark(0);
    m_ArmIntake = new Spark(4);
    m_AlgaePivot = new Spark(2);
    m_CoralLeft = new Spark(3);
    m_CoralRight = new Spark(1);
    //m_elevator = new SparkMax(16, kBrushless);
    //SparkMax neo = new SparkMax(16, kBrushless);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

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
  public void autonomousPeriodic() {}

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
      if (driver.getRawButton(PS4Controller.Button.kR2.value)){
        m_ArmIntake.set(.8);


    } else if (driver.getRawButton(PS4Controller.Button.kL2.value)) {
        m_ArmIntake.setInverted(true);
        m_ArmIntake.set(.8);

    } else {
      m_ArmIntake.stopMotor();

    } 
    //Algae Intake
      if (operator.getRawButton(PS4Controller.Button.kCircle.value)){
       m_AlgaeIntake.set(.8);

    } else if (operator.getRawButton(PS4Controller.Button.kSquare.value)){
      m_AlgaeIntake.setInverted(true);
      m_AlgaeIntake.set(.8);

    } else {
      m_AlgaeIntake.stopMotor();

    }
    //Algae pivot
      if (operator.getRawButton(PS4Controller.Button.kR1.value)){
      m_AlgaePivot.set(.5);

    } else if (operator.getRawButton(PS4Controller.Button.kL1.value)){
      m_AlgaePivot.setInverted(true);
      m_AlgaePivot.set(.5);
    
    } else {
      m_AlgaePivot.stopMotor();
    }


    // Coral Left/Right
    if (operator.getRawButton(PS4Controller.Button.kL2.value)){
      m_CoralLeft.setInverted(true);
      m_CoralLeft.set(.25);
      m_CoralRight.setInverted(true);
      m_CoralRight.set(.25);

    } else if (operator.getRawButton(PS4Controller.Button.kR2.value)){
      m_CoralLeft.setInverted(true);
      m_CoralLeft.set(.25);
      //m_CoralRight.setInverted(true);
      m_CoralRight.set(.25);

    } else {
      m_CoralLeft.stopMotor();
      m_CoralRight.stopMotor();    
    }
   
    
    // IntakePivot*
    if (driver.getRawButton(PS4Controller.Button.kL1.value)){
      intakepivot.set(.5);

    }else if (driver.getRawButton(PS4Controller.Button.kR1.value)){
      intakepivot.set(-.5);

    } else {
      intakepivot.stopMotor();
    }

    //elevator *
    if (operator.getRawAxis(PS4Controller.Axis.kLeftX.value) >= .1) {
      elevator.set(operator.getRawAxis(XboxController.Axis.kLeftX.value) * .6);
    } else if (operator.getRawAxis(XboxController.Axis.kRightX.value) < -.1) {
      elevator.set(-operator.getRawAxis(XboxController.Axis.kRightX.value) * .6);
    } else {
      elevator.stopMotor();
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
