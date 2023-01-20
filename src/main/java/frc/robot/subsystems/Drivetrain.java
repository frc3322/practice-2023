// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
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
 
  public final RelativeEncoder FLEncoder = motorFL.getEncoder();
  public final RelativeEncoder FREncoder = motorFR.getEncoder();
  public final RelativeEncoder BLEncoder = motorBL.getEncoder();
  public final RelativeEncoder BREncoder = motorBR.getEncoder();

  private final DifferentialDrive robotDrive = new DifferentialDrive(motorFL, motorFR);
  
  private final SlewRateLimiter accelLimit = new SlewRateLimiter(1.2);
  private final SlewRateLimiter turnLimit = new SlewRateLimiter(2);

  // This gets the default instance of NetworkTables that is automatically created.
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  
  // This gets the limelight table where data is stored from the limelight.
  NetworkTable limelightTable = inst.getTable("limelight");
  
  // Create gyro
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Create double for logging the yaw of the robot
  @Log private double pitch = 0;
  @Log private double roll = 0;
  @Log private double yaw = 0;

  // create double for logging the controller input
  @Log private double speed = -2;
  @Log private double turn = -2;

  @Log private double FLENCValue;
  @Log private double FRENCValue;
  @Log private double BLENCValue;
  @Log private double BRENCValue; 

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    motorFL.setInverted(true);
    motorFR.setInverted(false);
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
    //turn = 0.5 * turn + 0.5 * Math.pow(turn, 3);  // Weird math

    this.speed = speed;
    this.turn = turn;

    robotDrive.arcadeDrive(accelLimit.calculate(speed), turnLimit.calculate(turn), false);

    robotDrive.feed();
  }

  // Limelight Functions Start

  public void setPipeline(int pipelineNum){
    limelightTable
      //gets the "pipeline" entry from the limelight table
      .getEntry("pipeline")
      //sets the value of the pipeline entry to the parameter of the function
      .setNumber(pipelineNum);
  }

  // Limelight Functions End

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
    pitch = getPitch();
    roll = getRoll();
    yaw = getYaw();

    FLENCValue = FLEncoder.getPosition();
    FRENCValue = FREncoder.getPosition();
    BLENCValue = BLEncoder.getPosition();
    BRENCValue = BREncoder.getPosition();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getPitch(){
    return gyro.getPitch();
  }

  public double getRoll(){
    return gyro.getRoll();
  }
  
  public double getYaw() {
    return gyro.getYaw();
  }
}
