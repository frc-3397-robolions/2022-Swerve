// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private AHRS gyro;
  private SwerveModule backRight;
  private SwerveModule backLeft;
  private SwerveModule frontRight;
  private SwerveModule frontLeft;
  private SwerveDriveKinematics m_kinematics;
  double L = Constants.ROBOT_LENGTH;
  double W = Constants.ROBOT_WIDTH;
  /** Creates a new ExampleSubsystem. */
  public DriveTrain (SwerveModule backRight, SwerveModule backLeft, SwerveModule frontRight, SwerveModule frontLeft) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    gyro=new AHRS(Port.kMXP);
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

// Creating my kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
     
}
public void drive(double y, double x, double rot){
  ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x,y,rot,Rotation2d.fromDegrees(-gyro.getAngle()));
  //ChassisSpeeds speeds = new ChassisSpeeds(x,y,rot);
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
  frontLeft.drive(moduleStates[0]);
  frontRight.drive(moduleStates[1]);
  backLeft.drive(moduleStates[2]);
  backRight.drive(moduleStates[3]);
}
public void resetGyro(){
  gyro.zeroYaw();
}
public void zeroWheels(){
  frontLeft.zeroWheels();
  backLeft.zeroWheels();
  frontRight.zeroWheels();
  backRight.zeroWheels();
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
