// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

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

  //Creates colorSensor
  I2C colorSensor;

  //Color sensor values
  @Log int cData;
  @Log int rData;
  @Log int gData;
  @Log int bData;
  @Log int pData;

  // Gets Encoder Value
  @Log double encoderPosition = 0; 

  /** Creates a new Testmotor. */
  public TestMotor() {
    SmartDashboard.putData("Encoder To Position", getEncoderCommand(encoderToPosition.getSetpoint()));
    SmartDashboard.putData("encoderToPosition", encoderToPosition);

    colorSensor = new I2C(I2C.Port.kOnboard, 0x39);

    colorSensor.write(I2CConst.COMMAND_REGISTER_BIT | 0x00, 0b00000011);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getEncoderPosition();

    cData = readWordRegister(I2CConst.CDATA_REGISTER);
    rData = readWordRegister(I2CConst.RDATA_REGISTER);
    gData = readWordRegister(I2CConst.GDATA_REGISTER);
    bData = readWordRegister(I2CConst.BDATA_REGISTER);
    pData = readWordRegister(I2CConst.PDATA_REGISTER);
  }

  public double getEncoderPosition() {
    encoderPosition = testMotor_ENC.getPosition();
    return encoderPosition;
  }

  public int readWordRegister(int address){
    //Creates a ByteBuffer of size 2
    ByteBuffer buf = ByteBuffer.allocate(2);
    //Reads color sensor. 
    colorSensor.read(I2CConst.COMMAND_REGISTER_BIT | I2CConst.MULTI_BYTE_BIT | address, 2, buf);
    buf.order(ByteOrder.LITTLE_ENDIAN);
    return buf.getShort(0);
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
