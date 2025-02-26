// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.Percent;
import java.io.ObjectInputFilter.Config;
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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
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
  private TalonFX intakepivot = new TalonFX(8);
  private SparkMax elevator;
  // private SparkClosedLoopController maxPID = elevator.getClosedLoopController();
  // private SparkMaxConfig config = new SparkMaxConfig();
  // private SparkClosedLoopController m_controller = elevator.getClosedLoopController();
  

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
    // config.idleMode(IdleMode.kBrake);
    // config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    // config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    // elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // boolean isInverted = elevator.configAccessor.getInverted();
    // double positionFactor = elevator.configAccessor.encoder.getPositionConversionFactor();
    // double velocityFactor = elevator.configAccessor.encoder.getVelocityConversionFactor();
    // config.signals.primaryEncoderPositionPeriodMs(5);
    // double encoder = elevator.getEncoder().getPosition();
    // //SparkPIDController maxPID = elevator.getPIDController();
    // m_controller.setReference(encoder, SparkBase.ControlType.kPosition);

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
      if (driver.getRawButton(PS4Controller.Button.kR1.value)){
        m_ArmIntake.set(.5);


    } else if (driver.getRawButton(PS4Controller.Button.kL1.value)) {
        m_ArmIntake.set(-.5);

    } else {
      m_ArmIntake.stopMotor();

    } 
    //Algae Intake
      if (operator.getRawButton(PS4Controller.Button.kCircle.value)){
       m_AlgaeIntake.set(.5);

    } else if (operator.getRawButton(PS4Controller.Button.kSquare.value)){
      m_AlgaeIntake.set(-.5);

    } else {
      m_AlgaeIntake.stopMotor();

    }
    //Algae pivot
      if (operator.getRawButton(PS4Controller.Button.kR1.value)){
      m_AlgaePivot.set(.25);

    } else if (operator.getRawButton(PS4Controller.Button.kL1.value)){
      m_AlgaePivot.set(-.25);
    
    } else {
      m_AlgaePivot.stopMotor();
    }


    // Coral Left/Right
    if (operator.getRawButton(PS4Controller.Button.kL2.value)){
      m_CoralLeft.set(.25);
      m_CoralRight.set(.25);

    } else if (operator.getRawButton(PS4Controller.Button.kR2.value)){
      m_CoralLeft.set(-.25);
      m_CoralRight.set(-.25);

    } else {
      m_CoralLeft.stopMotor();
      m_CoralRight.stopMotor();    
    }
   
    
    // IntakePivot*
    if (driver.getRawAxis(PS4Controller.Axis.kL2.value) > .1) {
      intakepivot.set(driver.getRawAxis(PS4Controller.Axis.kL2.value) * .6);
    } else if (driver.getRawAxis(PS4Controller.Axis.kR2.value) > .1) {
      intakepivot.set(-driver.getRawAxis(PS4Controller.Axis.kR2.value) * .6);
    } else {
      intakepivot.stopMotor();
    }

    //elevator *
    // if (operator.getRawAxis(PS4Controller.Axis.kLeftX.value) >= .1) {
    //   elevator.set( .6);
    // } else if (operator.getRawAxis(PS4Controller.Axis.kLeftX.value) <= .1){
    //   elevator.set(-.6);
    // } else {
    //   elevator.stopMotor();
    // }
      if (operator.getRawButton(PS4Controller.Button.kShare.value)) {
        // m_controller.setReference(1.0, ControlType.kPosition);
        elevator.set(.75);
      } else if (operator.getRawButton(PS4Controller.Button.kOptions.value)) {
        // m_controller.setReference(-1.0, ControlType.kPosition);
        elevator.set(-.75);
      } else {
        // m_controller.setReference(0.0, ControlType.kPosition);
        elevator.set(0.0);
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
