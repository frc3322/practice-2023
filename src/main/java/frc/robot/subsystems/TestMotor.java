// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class TestMotor extends SubsystemBase implements Loggable {
  final CANSparkMax testMotor = new CANSparkMax(Constants.CAN.testMotor, MotorType.kBrushless);

  // Create Encoders
  private final RelativeEncoder testMotor_ENC = testMotor.getEncoder();

  // Gets Encoder Value
  @Log double encoderPosition = 0; 

  /** Creates a new Testmotor. */
  public TestMotor() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoderPosition = testMotor_ENC.getPosition();
    
  }

  public void setPower(double power) {
    testMotor.set(power);
  }

}
