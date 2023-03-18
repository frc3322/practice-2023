// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import java.util.function.BiConsumer;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;




public class AutonBalanceCommand extends CommandBase{
  
  private AutonBalance autoBalance;
  private BiConsumer<Double, Double> output;
  

  /** Creates a new AutoBalanceCommand. */
  public AutonBalanceCommand(AutonBalance autoBalance, BiConsumer<Double, Double> outputVolts, Subsystem... reqirements) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.autoBalance = requireNonNullParam(autoBalance, "AutoBalance", "RamseteCommand");
    this.output = requireNonNullParam(outputVolts, "outputVolts", "RamseteCommand");
    
    addRequirements(reqirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output.accept(autoBalance.autoBalanceRoutine(), autoBalance.autoBalanceRoutine());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autoBalance.isBalanced();
  }
}
