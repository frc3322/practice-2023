// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SysID;
import frc.robot.commands.TurnToAngle;
//import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TestMotor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Loggable{
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final TestMotor testMotor = new TestMotor();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);

    private final Command driveCommand = new RunCommand(
      () -> {
        double speed = MathUtil.applyDeadband(driverController.getLeftY(), 0.09);
        double turn = MathUtil.applyDeadband(driverController.getRightX(), 0.08);
        drivetrain.drive(speed, turn);
      }
      , drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    Logger.configureLoggingAndConfig(this, false);
    
    // Configure the trigger bindings
    configureBindings();

    drivetrain.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   
    driverController.a().whileTrue(new InstantCommand(
      () -> {
        drivetrain.setPipeline(0);
      }
    ));

    driverController.b().whileTrue(new InstantCommand(
      () -> {
        drivetrain.setPipeline(1);
      }
    ));

    driverController.y().onTrue(
        new TurnToAngle(90, drivetrain)
          .withTimeout(3)
    );
    driverController.x().whileTrue(new StartEndCommand(
      () -> {
        testMotor.setPower(0.02);
      }, 
      () -> {
        testMotor.setPower(0);
      }, testMotor)
    );

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }
public void updateLogger(){
  Logger.updateEntries();
}
  
   public Command getAutonomousCommand() {
//     var autoVoltageConstraint =
//       new DifferentialDriveVoltageConstraint(
//         new SimpleMotorFeedforward(SysID.ks, SysID.kv,SysID.ka), SysID.kDriveKinematics, 10);

//         TrajectoryConfig config =
//         new TrajectoryConfig(
//                 SysID.MaxSpeed,
//                 SysID.MaxAcceleration)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(SysID.kDriveKinematics)
//             // Apply the voltage constraint
//             .addConstraint(autoVoltageConstraint);

//     Trajectory tr = 
//     TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(0)),null , new Pose2d(0,1,new Rotation2d(0)), null);
//     RamseteCommand ramseteCommand =
//     new RamseteCommand(
//         tr,
//         drivetrain::getPose,
//         new RamseteController(SysID.kRamseteB, SysID.kRamseteZeta),
//         new SimpleMotorFeedforward(
//             SysID.ks,
//             SysID.kv,
//             SysID.ka),
//         SysID.kDriveKinematics,
//         drivetrain::getWheelSpeeds,
//         new PIDController(SysID.kp, 0, 0),
//         new PIDController(SysID.kp, 0, 0),
//        (double left, double right) -> drivetrain.tankDriveVolts
//       ,
//         drivetrain);

// // Reset odometry to the starting pose of the trajectory.
// drivetrain.resetOdometry(tr.getInitialPose());

// Run path following command, then stop at the end.
return null;
 
  }
}
