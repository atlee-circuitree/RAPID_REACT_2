// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretRotateAuto extends CommandBase {
  
  private final TurretSubsystem turret;
  private final LimeLightSubsystem limelight;
  private Timer timeoutTimer = new Timer();
  private double targetTimeout;
   
  public TurretRotateAuto(TurretSubsystem ts, LimeLightSubsystem ls, double timeout) {

    turret = ts;
    limelight = ls;
    targetTimeout = timeout;
    addRequirements(turret);

  }
 
  @Override
  public void initialize() {

  timeoutTimer.start();
  limelight.EnableLED();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Check Red/Blue and Green/Blue SparkMax led code
    if(limelight.HorizontalOffset() > .2){
      turret.turnTurret(limelight.HorizontalOffset() / 60);
    }
    else if(limelight.HorizontalOffset() < -.2){
      turret.turnTurret(limelight.HorizontalOffset() / 60);
    }
    else{
      turret.turnTurret(0);
    }

  }

  
  @Override
  public void end(boolean interrupted) {

    turret.turnTurret(0);

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