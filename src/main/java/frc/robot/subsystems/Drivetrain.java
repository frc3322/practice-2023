// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
  public final CANSparkMax motorFR = new CANSparkMax(Constants.CAN.FR, MotorType.kBrushless);
  public final CANSparkMax motorFL = new CANSparkMax(Constants.CAN.FL, MotorType.kBrushless);
  public final CANSparkMax motorBR = new CANSparkMax(Constants.CAN.BR, MotorType.kBrushless);
  public final CANSparkMax motorBL = new CANSparkMax(Constants.CAN.BL, MotorType.kBrushless);
  private final DifferentialDrive robotDrive = new DifferentialDrive(motorFL, motorFR);
  private final SlewRateLimiter accelLimit = new SlewRateLimiter(1.5);
  private final SlewRateLimiter turnLimit = new SlewRateLimiter(1.5);

  // create double for logging the controller input
  @Log double speedLog = -2;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    motorFL.setInverted(true);
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);


    motorFR.setIdleMode(IdleMode.kBrake);
    motorFL.setIdleMode(IdleMode.kBrake);
    motorBR.setIdleMode(IdleMode.kBrake);
    motorBL.setIdleMode(IdleMode.kBrake);

    motorFR.burnFlash();
    motorFL.burnFlash();
    motorBR.burnFlash();
    motorBL.burnFlash();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public void drive(double speed, double turn) {

    this.speedLog = speed;

    turn += Math.pow(turn, 3);
    speed *= .75;
    robotDrive.arcadeDrive(accelLimit.calculate(speed), turnLimit.calculate(turn));
    robotDrive.feed();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}