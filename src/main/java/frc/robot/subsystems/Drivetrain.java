// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import java.util.function.DoubleSupplier;


import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;
import frc.robot.Constants.SysID;
import frc.robot.commands.TurnToAngle;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


public class Drivetrain extends SubsystemBase implements Loggable {


  private Field2d fieldSim = new Field2d();


  public final CANSparkMax motorFR = new CANSparkMax(Constants.CAN.FR, MotorType.kBrushless);
  public final CANSparkMax motorFL = new CANSparkMax(Constants.CAN.FL, MotorType.kBrushless);
  public final CANSparkMax motorBR = new CANSparkMax(Constants.CAN.BR, MotorType.kBrushless);
  public final CANSparkMax motorBL = new CANSparkMax(Constants.CAN.BL, MotorType.kBrushless);
 
  public final RelativeEncoder FLEncoder = motorFL.getEncoder();
  public final RelativeEncoder FREncoder = motorFR.getEncoder();
  public final RelativeEncoder BLEncoder = motorBL.getEncoder();
  public final RelativeEncoder BREncoder = motorBR.getEncoder();


  private DifferentialDrivetrainSim drivetrainSimulator;
  private EncoderSim FLEncoderSim = new EncoderSim((Encoder) FLEncoder);
  private EncoderSim FREncoderSim = new EncoderSim((Encoder) FREncoder);
  private EncoderSim BLEncoderSim = new EncoderSim((Encoder) BLEncoder);
  private EncoderSim BREncoderSim = new EncoderSim((Encoder) BREncoder);
  private SimDouble gyroSim;


  private final DifferentialDrive robotDrive = new DifferentialDrive(motorFL, motorFR);
  private DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDoubleNEOPerSide, // 2 CIMs per side.
    KitbotGearing.k10p71,        // 10.71:1
    KitbotWheelSize.kSixInch,    // 6" diameter wheels.
    null                         // No measurement noise.
  );
 
  private final SlewRateLimiter accelLimit = new SlewRateLimiter(1.2);
  private final SlewRateLimiter turnLimit = new SlewRateLimiter(2);


  // This gets the default instance of NetworkTables that is automatically created.
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
 
  // This gets the limelight table where data is stored from the limelight.
  NetworkTable limelightTable = inst.getTable("limelight");
 
  //Array
  //@Log double[] roboPoseXYZRPY;
  //@Log double roboPoseX;
  //@Log double roboPoseY;
  //@Log double roboPoseZ;
  //@Log double roboPoseRoll;
  //@Log double roboPosePitch;
  //@Log double roboPoseYaw;
  //@Log double tid;
  // Array of the target apriltag in the coordinate system of the robot(robot perspective of apriltag)
  //@Log double[] targetPose_robotSpace;
  // Array of the robot in the coordinate system of the target apriltag (apriltag perspective of robot)
 // @Log double[] botPose_targetSpace;
  // Create gyro


  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final PIDController ddcontrol = new PIDController(.1, 0, 0.01);
 


  private final DifferentialDriveOdometry  odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), FLEncoder.getPosition(), FREncoder.getPosition());


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


  @Log private double FLPower;


  @Log double tx;


  @Log double driveToDistanceOutput;
 




  // @Config
  // public void tuneDistancePID( double p, double i, double d){
  //  ddcontrol.setPID(p, i, d);
  //  ddcontrol.setSetpoint(0);
  // }


  // @Config
  // public void tuneAnglePID( double p, double i, double d){
  //  anglecontrol.setPID(p, i, d);
  //  anglecontrol.setSetpoint(0);
  // }


 


  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    motorFL.setInverted(true);
    motorFR.setInverted(false);
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);


    SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> {gyro.reset();}, this));
    SmartDashboard.putData("drive to distance", getDriveEncDistanceCommandFL(0));
   




    motorFR.setIdleMode(IdleMode.kCoast);
    motorFL.setIdleMode(IdleMode.kCoast);
    motorBR.setIdleMode(IdleMode.kCoast);
    motorBL.setIdleMode(IdleMode.kCoast);


    motorFR.burnFlash();
    motorFL.burnFlash();
    motorBR.burnFlash();
    motorBL.burnFlash();


    SmartDashboard.putData("Field", fieldSim);


   


  }


 
  @Override
  public void periodic() {
    pitch = getPitch();
    roll = getRoll();
    yaw = getYaw();


    FLPower = motorFL.getBusVoltage();


    updatePose();


    odometry.update(new Rotation2d(getYaw()), FLENCValue, FRENCValue);


    //tid = limelightTable.getEntry("tid").getValue().getDouble();
    //targetPose_robotSpace = limelightTable.getEntry("targetpose_robotspace").getValue().getDoubleArray();
    // botPose_targetSpace = limelightTable.getEntry("botpose_targetspace").getValue().getDoubleArray();


    FLENCValue = FLEncoder.getPosition();
    FRENCValue = FREncoder.getPosition();
    BLENCValue = BLEncoder.getPosition();
    BRENCValue = BREncoder.getPosition();


    //tx = limelightTable.getEntry("tx").getValue().getDouble();


    odometry.update(gyro.getRotation2d(), FLEncoder.getPosition(), FREncoder.getPosition());
  }


  public void updateOutputLog(double output) {
    driveToDistanceOutput = output;
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    fieldSim.setRobotPose(getPose2d());
    //Set the inputs to the system
    driveSim.setInputs(motorFL.get(), motorFL.get());
    //Update the simulation
    driveSim.update(0.02);
    //Update the sensors
    FLEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    FLEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    FREncoderSim.setDistance(driveSim.getRightPositionMeters());
    FREncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    gyroSim.set(driveSim.getHeading().getDegrees());
  }


  public void drive(double speed, double turn) {
    //turn = 0.5 * turn + 0.5 * Math.pow(turn, 3);  // Weird math


    this.speed = speed;
    this.turn = turn;


    robotDrive.arcadeDrive(accelLimit.calculate(speed), turnLimit.calculate(turn), false);


    robotDrive.feed();
  }


  public Pose2d getPose2d(){
    return odometry.getPoseMeters();
  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(FLEncoder.getVelocity(), FREncoder.getVelocity());
  }


  // Limelight Functions Start


  @Log
  public double GetTX(){
    return tx;
  }


  public double getDistance() {
    return FREncoder.getPosition();
  }


  public void resetEncoders() {
    FREncoder.setPosition(0);
    FLEncoder.setPosition(0);
    BREncoder.setPosition(0);
    BLEncoder.setPosition(0);
  }


  public void setPipeline(int pipelineNum){
    limelightTable
      //gets the "pipeline" entry from the limelight table
      .getEntry("pipeline")
      //sets the value of the pipeline entry to the parameter of the function
      .setNumber(pipelineNum);
  }


  // Limelight Functions End


  public void updatePose(){
    // roboPoseXYZRPY = limelightTable.getEntry("botpose").getValue().getDoubleArray();
    // if(roboPoseXYZRPY.length > 0){
    //   roboPoseX = roboPoseXYZRPY[0];
    //   roboPoseY = roboPoseXYZRPY[1];
    //   roboPoseZ = roboPoseXYZRPY[2];
    //   roboPoseRoll = roboPoseXYZRPY[3];
    //   roboPosePitch = roboPoseXYZRPY[4];
    //   roboPoseYaw = roboPoseXYZRPY[5];
    //}
  }


  public double getPitch(){
    return gyro.getPitch();
  }


  public double getRoll(){
    return gyro.getRoll();
  }
 
  @Log
  public double getYaw() {
    return gyro.getYaw();
  }


  public InstantCommand resetGyro() {
    return new InstantCommand(() -> {
      gyro.reset();
    }, this);
  }
 
  public PIDCommand getDriveEncDistanceCommandFL(double setpoint){
    PIDCommand c  =  new PIDCommand(ddcontrol, FLEncoder::getPosition, setpoint, (output) -> motorFL.set(output));
    //c.getController().setTolerance(5);
    return c;
  }
  // public PIDCommand getDriveEncDistanceCommandFR(double setpoint){
  //   PIDCommand c  =  new PIDCommand(ddcontrol, FREncoder::getPosition, setpoint, (output) -> motorFR.set(output));
  //   //c.getController().setTolerance(5);
  //   return c;
  // }
  // public void getDriveEncDistanceCommand(double setpoint){
  //   PIDCommand rightCommand  =  new PIDCommand(ddcontrol, FREncoder::getPosition, setpoint, (output) -> motorFR.set(output));
  //   PIDCommand leftCommand  =  new PIDCommand(ddcontrol, FLEncoder::getPosition, setpoint, (output) -> motorFL.set(output));
  //   rightCommand.execute();
  //   leftCommand.execute();
  // }


  public void tankDriveVolts(double left, double right){
    motorFL.setVoltage(left);
    motorFR.setVoltage(right);
  }




  public void resetOdometry(Pose2d initialPose) {
  }


  public void putTrajOnFieldWidget(Trajectory trajectory, String label) {
    fieldSim.getObject(label).setTrajectory(trajectory);
  }


  // public Command getAuto(){
   
  // }
}



