// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.TurretSubsystem;

public class RunShooter extends CommandBase {
  
  private Timer timer = new Timer();
  private double timeout;
  private final TurretSubsystem turret;
  private double velocity;

  public RunShooter(double targetVelocity, double timeoutSeconds, TurretSubsystem ts) {

    timeout = timeoutSeconds;
    turret = ts;
    velocity = targetVelocity;
    addRequirements(turret);

  }
 
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    turret.runTurretWithVelocity(velocity);

    
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeout);    
  }
}
