// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  
  DoubleSolenoid climbLeft;
  DoubleSolenoid climbRight;
  DoubleSolenoid shooterPiston;
  
  public Pneumatics() {

    climbLeft = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, Constants.climbLeftPnumaticDeploy, Constants.climbLeftPnumaticRetract);
    climbRight = new DoubleSolenoid(15, PneumaticsModuleType.REVPH, Constants.climbRightPnumaticDeploy, Constants.climbRightPnumaticRetract);
    shooterPiston = new DoubleSolenoid(15 , PneumaticsModuleType.REVPH, Constants.shootPnumaticDeploy, Constants.shootPnumaticRetract);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbPistonsUp(){
    climbLeft.set(Value.kForward);
    climbRight.set(Value.kForward);
  }
  public void climbPistonsDown(){
    climbLeft.set(Value.kReverse);
    climbRight.set(Value.kReverse);
  }
  public void shooterUp(){
    shooterPiston.set(Value.kForward);
  }
  public void shooterDown(){
    shooterPiston.set(Value.kReverse);
  }
}
