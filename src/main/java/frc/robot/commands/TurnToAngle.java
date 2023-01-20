// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends PIDCommand implements Loggable{
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   * 
   */

    static double p;
    static double i;
    static double d;

   @Config
   public void tuneTTA(double newP, double newI, double newD){
    p = newP;
    i = newI;
    d = newD;
   }

  public TurnToAngle(double targetAngleDegrees, Drivetrain drive) {
    super(
        new PIDController(p, i, d),
        // Close loop on heading
        drive::getYaw,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.drive(0, output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}