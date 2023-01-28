// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** A command that will turn the robot to the specified angle. */
public class DriveToDistance extends PIDCommand {
  
  public DriveToDistance(double targetDistance, Drivetrain drive) {
    super(
      new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD),
        // Close loop on heading
        drive::getDistance,
        // Set reference to target
        drive.getDistance() + targetDistance,
        // Pipe output to turn robot
        output -> drive.drive(output, 0),
        // Require the drive
        drive);
    
    SmartDashboard.putData("Distance Controller", getController());
    

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(0, 60);
    
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kDriveToleranceDeg, DriveConstants.kDriveRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}