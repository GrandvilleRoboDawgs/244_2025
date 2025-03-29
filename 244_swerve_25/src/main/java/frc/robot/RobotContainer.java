// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.DriveForwardCommand;
// import frc.robot.subsystems.DrivetrainSubsystem;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS4Controller joystick = new CommandPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed *.75) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed *.75) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void driveBack() {
        // drivetrain.sysIdDynamic(Direction.kReverse);
        // drivetrain.setControl(drive.withVelocityX(-.5 * 4.73));
        drivetrain.applyRequest(() -> drive.withVelocityX(-.5 * 4.73));
        drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-.5, 0)));
    }

    public Command getAutonomousCommand() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        double kP = 0.008; // Tune this value
        double strafeSpeed1 = (tx - 3) * kP;
        double strafeSpeed2 = (tx - 2) * kP;



        // Right side auto [L4->feeder->L4] (RED SIDE!!!!!!!!!!!!!!!)
            // return new SequentialCommandGroup(
            //     //1
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.28*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(-.11*MaxAngularRate)).withTimeout(1.2),
            //     //2
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.13*MaxSpeed)
            //     .withVelocityY(.14*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(2),
            //     //3
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(.5),
            //     //4
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.35*MaxSpeed)
            //     .withVelocityY(-.25*MaxSpeed)
            //     .withRotationalRate(-.1*MaxAngularRate)).withTimeout(2.3),
            //     //5
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.25*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1),
            //     //6
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1.5),

            // //L4 across from FS
            //     // //7
            //     // drivetrain.applyRequest(()->
            //     // drive.withVelocityX(.25*MaxSpeed)
            //     // .withVelocityY(.25*MaxSpeed)
            //     // .withRotationalRate(0)).withTimeout(1));

            // //L4 across from DS
            //     //7
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.15*MaxSpeed)
            //     .withVelocityY(.26*MaxSpeed)
            //     .withRotationalRate(-.1*MaxAngularRate)).withTimeout(1.55),
            //     //8
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(.22*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(1),
            //     //9
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.24*MaxSpeed)
            //     .withVelocityY(.1*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(1));

        

        //Left side auto [L4->feeder->L4] (RED SIDE!!!!!!!!!!!!!!)
            // return new SequentialCommandGroup(
            //     //1
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.25*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(.11*MaxAngularRate)).withTimeout(1.2),
            //     //2
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.14*MaxSpeed)
            //     .withVelocityY(-.14*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(2),
            //     //3
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(.5),
            //     //4
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.35*MaxSpeed)
            //     .withVelocityY(.25*MaxSpeed)
            //     .withRotationalRate(.1*MaxAngularRate)).withTimeout(2),
            //     //5
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.25*MaxSpeed)
            //     .withVelocityY(-.05*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(1),
            //     //6
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1.5),

            // //L4 across from FS
            //     //7
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.27*MaxSpeed)
            //     .withVelocityY(-.24*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(1.95));


            // // L4 across from DS
            //     // //7
            //     // drivetrain.applyRequest(()->
            //     // drive.withVelocityX(.15*MaxSpeed)
            //     // .withVelocityY(-.26*MaxSpeed)
            //     // .withRotationalRate(.1*MaxAngularRate)).withTimeout(1.55),
            //     // //8
            //     // drivetrain.applyRequest(()->
            //     // drive.withVelocityX(0)
            //     // .withVelocityY(-.20*MaxSpeed)
            //     // .withRotationalRate(0)).withTimeout(1),
            //     // //9
            //     // drivetrain.applyRequest(()->
            //     // drive.withVelocityX(.24*MaxSpeed)
            //     // .withVelocityY(-.11*MaxSpeed)
            //     // .withRotationalRate(0)).withTimeout(.9));



        // Right side auto [L4->feeder->L4] (BLUE SIDE!!!!!!!!!!!!!!!)
            // return new SequentialCommandGroup(
            //     //1
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.25*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(-.11*MaxAngularRate)).withTimeout(1.2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.11*MaxSpeed)
            //     .withVelocityY(-.14*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(.5),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.35*MaxSpeed)
            //     .withVelocityY(.25*MaxSpeed)
            //     .withRotationalRate(-.1*MaxAngularRate)).withTimeout(2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.25*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1.5),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.15*MaxSpeed)
            //     .withVelocityY(-.26*MaxSpeed)
            //     .withRotationalRate(-.1*MaxAngularRate)).withTimeout(1.55),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(-.22*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(1),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.23*MaxSpeed)
            //     .withVelocityY(-.11*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(.9));



        // Left side auto [L4->feeder->L4] (BLUE SIDE!!!!!!!!!!!!!!)
            // return new SequentialCommandGroup(
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.25*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(.11*MaxAngularRate)).withTimeout(1.2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.11*MaxSpeed)
            //     .withVelocityY(.14*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(.5),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.35*MaxSpeed)
            //     .withVelocityY(-.25*MaxSpeed)
            //     .withRotationalRate(.1*MaxAngularRate)).withTimeout(2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.25*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1.5),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.15*MaxSpeed)
            //     .withVelocityY(.26*MaxSpeed)
            //     .withRotationalRate(.1*MaxAngularRate)).withTimeout(1.55),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(.22*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(1),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.23*MaxSpeed)
            //     .withVelocityY(.11*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(.9));



        // middle auto to score L4 (RED SIDE!!!!!!!!!!!!)
            // return new SequentialCommandGroup(
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.25*MaxSpeed)
            //     .withVelocityY(strafeSpeed1*MaxSpeed)
            //     .withRotationalRate(0)).withTimeout(1.2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(10));



        // middle auto to score L4 (BLUE SIDE!!!!!!!!!!!!)
            return new SequentialCommandGroup(
                drivetrain.applyRequest(()->
                drive.withVelocityX(.25*MaxSpeed)
                .withVelocityY(-strafeSpeed1*MaxSpeed)
                .withRotationalRate(0)).withTimeout(1.2),
                drivetrain.applyRequest(()->
                drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)).withTimeout(15));



        // Drive forward auto (RED SIDE!!!!!!!!!!!)
            // return new SequentialCommandGroup(
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(.25*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1.2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(10));



        // Drive forward auto (BLUE SIDE!!!!!!!!!!!)
            // return new SequentialCommandGroup(
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(-.25*MaxSpeed)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(1.2),
            //     drivetrain.applyRequest(()->
            //     drive.withVelocityX(0)
            //     .withVelocityY(0)
            //     .withRotationalRate(0)).withTimeout(10));
            
            
        
            //new DriveForwardCommand(drivetrain, 0.5, 2) // Drive forward at 50% speed for 2 seconds
            // // Add more commands here if needed
        
        // return new PathPlannerAuto("New Auto");
        // //return Commands.print("No autonomous command configured");
    }
}
