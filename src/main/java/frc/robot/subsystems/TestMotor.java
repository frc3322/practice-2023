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
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestMotor extends SubsystemBase implements Loggable {
  
  final CANSparkMax testMotor = new CANSparkMax(Constants.CAN.testMotor, MotorType.kBrushless);
  final Servo newContinuousServo = new Servo(Constants.PWM.S1);
  final Servo newServo =  new Servo(Constants.PWM.S2);

  int servoCounter = 0;
  int servoContinuousCounter = 0;

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
    public void testContinuousServo(){
      //(sets to 0, then to 1, then back to 0 using degrees and whole values to check that they are same)
      if (servoContinuousCounter==0){
        newContinuousServo.set(0);
        servoContinuousCounter+=1;
      }
      else if (servoContinuousCounter==1){
        newContinuousServo.setAngle(180);
        servoContinuousCounter+=1;
      }
      else if (servoContinuousCounter==2){
        newContinuousServo.setAngle(0);
        servoContinuousCounter+=1;
      }
      else if (servoContinuousCounter==3){
        newContinuousServo.set(1.0);
        servoContinuousCounter+=1;
      }
      else if (servoContinuousCounter==4){
        //set 0 does not stop, must use setAngle(90) to stop
        newContinuousServo.setAngle(90);
        servoContinuousCounter=0;
      }

    }
    public void testServo(){
      //(sets to 0, then to 1, then back to 0 using degrees and whole values to check that they are same)
      if (servoCounter==0){
        newServo.set(0);
        servoCounter=1;
      }
      else if (servoCounter==1){
        newServo.setAngle(180);
        servoCounter=2;
      }
      else if (servoCounter==2){
        newServo.setAngle(0);
        servoCounter=3;
      }
      else if (servoCounter==3){
        newServo.set(1.0);
        servoCounter=0;
        
      }

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
    //orders bytes  from low to high
    buf.order(ByteOrder.LITTLE_ENDIAN);
    //returns 2 bytes after paramter position (all of the bytes in this situation)
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
