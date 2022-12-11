// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private SwerveModule backRight;
  private SwerveModule backLeft;
  private SwerveModule frontRight;
  private SwerveModule frontLeft;
  double L = Constants.ROBOT_LENGTH;
  double W = Constants.ROBOT_WIDTH;
  /** Creates a new ExampleSubsystem. */
  public DriveTrain (SwerveModule backRight, SwerveModule backLeft, SwerveModule frontRight, SwerveModule frontLeft) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
     
}
public void drive(double x1, double y1, double x2){
  double r = Math.sqrt ((L * L) + (W * W));
  y1 *= -1;

  double a = x1 - x2 * (L / r);
  double b = x1 + x2 * (L / r);
  double c = y1 - x2 * (W / r);
  double d = y1 + x2 * (W / r);


  double backRightSpeed = Math.sqrt ((a * a) + (d * d));
  double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
  double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
  double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

  double backRightAngle = Math.atan2 (a, d) / Math.PI;
  double backLeftAngle = Math.atan2(a, c) / Math.PI;
  double frontRightAngle = Math.atan2(b, d) / Math.PI;
  double frontLeftAngle = Math.atan2(b, c) / Math.PI;

  backRight.drive(backRightSpeed, backRightAngle);
  backLeft.drive(backLeftSpeed, backLeftAngle);
  frontLeft.drive(frontLeftSpeed, frontLeftAngle);
  frontRight.drive(frontRightSpeed, frontRightAngle);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
