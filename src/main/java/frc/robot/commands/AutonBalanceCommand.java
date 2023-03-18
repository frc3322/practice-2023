// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.util.function.BiConsumer;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;




public class AutonBalanceCommand extends CommandBase implements Loggable{
  
  private Drivetrain drivetrain;
  private BiConsumer<Double, Double> output;

  @Log private int state;
  private int debounceCount;
  private double robotSpeedSlow;
  private double robotSpeedFast;
  private double onChargeStationDegree;
  private double levelDegree;
  private double debounceTime;

  /** Creates a new AutoBalanceCommand. */
  public AutonBalanceCommand( 
    Drivetrain drivetrain, 
    BiConsumer<Double, Double> outputVolts, 
    Subsystem... reqirements) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    state = 0;
    debounceCount = 0;

    /**********
     * CONFIG *
     **********/
    // Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 2;

    // Speed the robot drives while balancing itself on the charge station.
    // Should be roughly half the fast speed, to make the robot more accurate,
    // default = 0.2
    robotSpeedSlow = .8;
    

    // Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 17;

    // Angle where the robot can assume it is level on the charging station
    // Used for exiting the drive forward sequence as well as for auto balancing,
    // default = 6.0
    levelDegree = 11; //3

    // Amount of time a sensor condition needs to be met before changing states in
    // seconds
    // Reduces the impact of sensor noice, but too high can make the auto run
    // slower, default = 0.2
    debounceTime = 0.3;
    
    this.drivetrain = requireNonNullParam(drivetrain, "Drivetrain", "AutonBalanceCommand");
    this.output = requireNonNullParam(outputVolts, "outputVolts", "AutonBalanceCommand");
    
    addRequirements(reqirements);
  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  public double getPitch(){
    return -drivetrain.getPitch();
  }

  public double autoBalanceRoutine() {
    switch (state) {
        // drive forwards to approach station, exit when tilt is detected
        case 0:
            if (getPitch() > onChargeStationDegree) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(debounceTime)) {
                state = 1;
                debounceCount = 0;
                return robotSpeedSlow;
            }
            return robotSpeedFast;
        // driving up charge station, drive slower, stopping when level
        case 1:
            if (getPitch() < levelDegree) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(debounceTime)) {
                state = 2;
                debounceCount = 0;
                return 0;
            }
            return robotSpeedSlow;
        // on charge station, stop motors and wait for end of auto
        case 2:
            if (Math.abs(getPitch()) <= levelDegree / 2) {
                debounceCount++;
            }
            if (debounceCount > secondsToTicks(debounceTime)) {
                state = 4;
                debounceCount = 0;
                return 0;
            }
            if (getPitch() >= levelDegree) {
                return 0.1;
            } else if (getPitch() <= -levelDegree) {
                return -0.1;
            }
        case 3:
            return 0;
    }
    return 0;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output.accept(autoBalanceRoutine(), autoBalanceRoutine());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autoBalanceRoutine() == 0;
  }
}
