// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pathweaver extends SubsystemBase {

  private double[][] coordinates = {{36,-1.95}, {2,-1}, {1,7000}, {6,4}};

  private double A;
  private double B;
  private double C;
  private double D;

  private SimpleMatrix XmatrixConstructor = new SimpleMatrix(4, 4);
  private SimpleMatrix YmatrixConstructor = new SimpleMatrix(4, 4);
    
  private Matrix X = new Matrix<>(XmatrixConstructor);
  private Matrix Y = new Matrix<>(YmatrixConstructor);



  /** Creates a new Pathweaver. */
  public Pathweaver() {  

    /*
      MATRIX X (P1X^3 = point 1 X value cubed):
      [P1X^3, P1X^2, P1X, 1]
      [P2X^3, P2X^2, P2X, 1]
      [P3X^3, P3X^2, P3X, 1]
      [P4X^3, P4X^3, P4X, 1]
    */

    
    //Row 1
    X.set(0, 0, Math.pow(coordinates[0][0], 3));
    X.set(0, 1, Math.pow(coordinates[0][0], 2));
    X.set(0, 2, Math.pow(coordinates[0][0], 1));
    X.set(0, 3, Math.pow(coordinates[0][0], 0));

    //Row 2
    X.set(1, 0, Math.pow(coordinates[1][0], 3));
    X.set(1, 1, Math.pow(coordinates[1][0], 2));
    X.set(1, 2, Math.pow(coordinates[1][0], 1));
    X.set(1, 3, Math.pow(coordinates[1][0], 0));

    //Row 3
    X.set(2, 0, Math.pow(coordinates[2][0], 3));
    X.set(2, 1, Math.pow(coordinates[2][0], 2));
    X.set(2, 2, Math.pow(coordinates[2][0], 1));
    X.set(2, 3, Math.pow(coordinates[2][0], 0));

    //Row 4
    X.set(3, 0, Math.pow(coordinates[3][0], 3));
    X.set(3, 1, Math.pow(coordinates[3][0], 2));
    X.set(3, 2, Math.pow(coordinates[3][0], 1));
    X.set(3, 3, Math.pow(coordinates[3][0], 0));

    /*
      MATRIX Y (P1Y = point 1 Y value):
      [P1Y]
      [P2Y]
      [P3Y]
      [P4Y]
    */

    
    //Row 1
    Y.set(0, 0, coordinates[0][1]);
    Y.set(1, 0, coordinates[1][1]);
    Y.set(2, 0, coordinates[2][1]);
    Y.set(3, 0, coordinates[3][1]);
    
    Matrix invX = X.inv();
    Matrix Z = invX.times(Y);
    
    SmartDashboard.putNumber("A", Z.get(0, 0));
    SmartDashboard.putNumber("B", Z.get(1, 0));
    SmartDashboard.putNumber("C", Z.get(2, 0));
    SmartDashboard.putNumber("D", Z.get(3, 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
