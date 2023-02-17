// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private SparkMaxPIDController pidController;
  public RelativeEncoder encoder;
  private String name;
  private double MAX_VOLTS = Constants.SWERVE_MAX_VOLTS;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new SwerveModule. */
  public SwerveModule(int angleMotorPort, int speedMotorPort, String name) {
    this.angleMotor = new CANSparkMax(angleMotorPort, MotorType.kBrushless);
    angleMotor.restoreFactoryDefaults();
    this.speedMotor = new CANSparkMax(speedMotorPort, MotorType.kBrushless);
    speedMotor.restoreFactoryDefaults();
    this.pidController = angleMotor.getPIDController();
    this.encoder = angleMotor.getEncoder();
    encoder.setPositionConversionFactor(360 / 21);
    this.name = name;
    pidController.setFeedbackDevice(encoder);
    kP = 0.01;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void drive(SwerveModuleState state) {
    var optimizedstate = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(-encoder.getPosition()));
    double targetAngle = -optimizedstate.angle.getDegrees();
    speedMotor.set(optimizedstate.speedMetersPerSecond / 4);
    pidController.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber(name, encoder.getPosition());
  }

  public void zeroWheels() {
    encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
    SmartDashboard.putNumber("key", encoder.getVelocity());
  }
}
