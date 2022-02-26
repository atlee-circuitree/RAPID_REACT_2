// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class ShooterPistonToggle extends CommandBase {
  
  private Timer timer = new Timer();
  private double timeout = 0.5;
  private final Pneumatics pneumatics;

  public ShooterPistonToggle(Pneumatics ps){
    
    pneumatics = ps;
    
    addRequirements(pneumatics);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();
    pneumatics.shooterUp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    pneumatics.shooterDown();
    timer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return timer.hasElapsed(timeout);
  
  }
}
