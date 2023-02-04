// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.I2CConst;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMotor extends SubsystemBase implements Loggable {
  
  final CANSparkMax testMotor = new CANSparkMax(Constants.CAN.testMotor, MotorType.kBrushless);

  // Create Encoders
  private final RelativeEncoder testMotor_ENC = testMotor.getEncoder();

  // Creates PID
  public final PIDController encoderToPosition = new PIDController(0, 0, 0);

  

  // Gets Encoder Value
  @Log double encoderPosition = 0; 

  /** Creates a new Testmotor. */
  public TestMotor() {
    SmartDashboard.putData("Encoder To Position", getEncoderCommand(encoderToPosition.getSetpoint()));
    SmartDashboard.putData("encoderToPosition", encoderToPosition);

    I2C colorSensor = new I2C(I2C.Port.kOnboard, 0x39);

    colorSensor.write(I2CConst.COMMAND_REGISTER_BIT | 0x00, 0b00000011);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getEncoderPosition();
    
  }

  public double getEncoderPosition() {
    encoderPosition = testMotor_ENC.getPosition();
    return encoderPosition;
  }

  public void setPower(double power) {
    testMotor.set(power);
  }

  public PIDCommand getEncoderCommand(double setpoint){
    
    testMotor_ENC.setPosition(0);
    PIDCommand  d =  new PIDCommand(encoderToPosition, this::getEncoderPosition, setpoint, output -> setPower(output) , this);
    //d.getController().setTolerance(0.0000000);
    d.getController().enableContinuousInput(-0.5, 0.5);
    return d;
  }

}
