// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TurretSubsystem;

public class RunFeederAuto extends CommandBase {
   
  private final FeederSubsystem feeder;
  private final Pneumatics pnuematic;
  private double targetSpeed; 
  private Timer timeoutTimer = new Timer();
  private double targetTimeout;

  public RunFeederAuto(double speed, FeederSubsystem fs, Pneumatics ps, double timeout) {
 
    feeder = fs;
    pnuematic = ps;
    targetSpeed = speed;
    targetTimeout = timeout;
    addRequirements(feeder);

  }
 
  @Override
  public void initialize() {

  timeoutTimer.start();
  feeder.runFeeder(targetSpeed);

  }

  @Override
  public void execute() {
 
  }

  @Override
  public void end(boolean interrupted) {
    
  }
 
  @Override
  public boolean isFinished() {

    if (timeoutTimer.get() < targetTimeout) {

      return false;

    } else {

      return true;

    }

  } 

}