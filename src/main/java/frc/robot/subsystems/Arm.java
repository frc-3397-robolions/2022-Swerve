// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase {
  private PWMVictorSPX motor1;
  private PWMVictorSPX motor2;
  private PWMVictorSPX motor3;
  private CANSparkMax grabberMotor;
  private PIDController firstJointPID;
  private PIDController secondJointPID;
  private Encoder firstJointEncoder;
  private Encoder secondJointEncoder;
  private AnalogInput testMeasDogAnalog;
  private DigitalInput limitSwitch;
  /** Creates a new Arm. */
  public Arm() {
    motor1 = new PWMVictorSPX(Constants.ARM_MOTOR_1_ID);
    motor2 = new PWMVictorSPX(Constants.ARM_MOTOR_2_ID);
    motor3 = new PWMVictorSPX(5);
    firstJointPID = new PIDController(0.025, 0.01, 0);
    firstJointEncoder = new Encoder(9, 1);
    secondJointPID = new PIDController(0.1, 0, 0);
    secondJointEncoder = new Encoder(2, 3);
    firstJointEncoder.setDistancePerPulse(0.0439453125);
    /*
     * AXISENSE-1 SERIES * VOLTAGE TILT SENSOR
     * Yellow-Black: Ground
     * Brown-Red: Analog Signal - Rio Analog In
     * White-Red: Voltage (8V+) - 12V VRM
     */
    testMeasDogAnalog = new AnalogInput(0);
    limitSwitch = new DigitalInput(0);
  }

  public void setBottomAngle(double angleDeg){
    firstJointPID.setSetpoint(angleDeg);
  }
  public void setTopAngle(double angleDeg){
    secondJointPID.setSetpoint(angleDeg);
  }
  public void zeroBottomEncoder(){
    firstJointEncoder.reset();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var bottompower = MathUtil.clamp(
      firstJointPID.calculate(firstJointEncoder.getDistance()),
      -0.75,
      0.75);
    motor1.set(bottompower);
    motor2.set(bottompower);
    // motor3.set(secondJointPID.calculate(secondJointEncoder.getDistance()));
    double dog1Offset = SmartDashboard.getNumber("DOG1 value", 2043);
    double dog1Divisor = SmartDashboard.getNumber("DOG1 divisor", 9.0);
    SmartDashboard.putNumber("Power", bottompower);
    SmartDashboard.putNumber("Encoder Position", firstJointEncoder.getDistance());
    SmartDashboard.putNumber("DOG1 Angle", Math.round((testMeasDogAnalog.getValue()-dog1Offset)/dog1Divisor));
    SmartDashboard.putBoolean("limit Switch", !limitSwitch.get());
    // SmartDashboard.putNumber("DOG1 raw", testMeasDogAnalog.getValue());
    //Flat=2050 upside down = 3665 90 degrees = 2042
  }
}
