// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Pneumatics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class KickoutFeeder extends CommandBase {
  
  private final Pneumatics pnuematic;
  private boolean Up; 
  private Timer timeoutTimer = new Timer();
  private double targetTimeout;

  public KickoutFeeder(boolean up, Pneumatics ps, double timeout) {
 
    pnuematic = ps;
    addRequirements(pnuematic);
    targetTimeout = timeout;
    up = Up;

  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timeoutTimer.start();

    if (Up == true) {

      pnuematic.kickoutRetract();
  
      } else {
  
      pnuematic.kickout();
  
      }

  }

  @Override
  public void execute() {

    
  }

  @Override
  public void end(boolean interrupted) {

   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timeoutTimer.get() < targetTimeout) {

      return false;

    } else {

      return true;

    }
  }

}
