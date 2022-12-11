// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax angleMotor;
  private CANSparkMax speedMotor;
  private PIDController pidController;
  private RelativeEncoder encoder;
  private double MAX_VOLTS = Constants.SWERVE_MAX_VOLTS;
  /** Creates a new SwerveModule. */
  public SwerveModule(int angleMotorPort, int speedMotorPort) {
    this.angleMotor = new CANSparkMax(angleMotorPort, MotorType.kBrushless);
    this.speedMotor = new CANSparkMax(speedMotorPort, MotorType.kBrushless);
    this.pidController = new PIDController(1, 0, 0);
    this.encoder = angleMotor.getEncoder();
  }
  public void drive (double speed, double angle) {
    speedMotor.setVoltage(speed);

    double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // @Optimization offset can be calculated here.
    if (setpoint < 0) {
        setpoint = MAX_VOLTS + setpoint;
    }
    if (setpoint > MAX_VOLTS) {
        setpoint = setpoint - MAX_VOLTS;
    }

    pidController.setSetpoint(setpoint);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angleMotor.setVoltage(pidController.calculate(encoder.getPosition()));
  }
}
