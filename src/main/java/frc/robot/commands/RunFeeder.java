// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class RunFeeder extends CommandBase {
  
  private final FeederSubsystem feeder;

  public RunFeeder(FeederSubsystem fs) {

    feeder = fs;
    addRequirements(feeder);

  }
 
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    feeder.runFeeder(.65);

  }

  
  @Override
  public void end(boolean interrupted) {
 
    feeder.runFeeder(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
