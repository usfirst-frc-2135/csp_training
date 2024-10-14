// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType; 
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final MotorController m_leftRearMotor = new WPI_TalonSRX(1);
  private final MotorController m_leftFrontMotor = new WPI_TalonSRX(2);
  private final MotorController m_rightRearMotor = new WPI_TalonSRX(3);
  private final MotorController m_rightFrontMotor = new WPI_TalonSRX(4);
  private XboxController controller; 
  SlewRateLimiter xFilter = new SlewRateLimiter(2);
  SlewRateLimiter yFilter = new SlewRateLimiter(2);

  MotorControllerGroup m_leftMotorGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  MotorControllerGroup m_rightMotorGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);
  
  private final DifferentialDrive m_robotDrive = 
  new DifferentialDrive(m_leftMotorGroup::set, m_rightMotorGroup::set);

  
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotorGroup);
    SendableRegistry.addChild(m_robotDrive, m_rightMotorGroup);
  }

  @Override
  public void robotInit() {
    m_rightMotorGroup.setInverted(true);
    controller = new XboxController(0);

    controller.setRumble(RumbleType.kLeftRumble, 1.0);
    controller.setRumble(RumbleType.kRightRumble, 1.0);
  }

  @Override
  public void teleopPeriodic() {    
    //turn off rumble in teleop periodic 
    controller.setRumble(RumbleType.kLeftRumble, 0.0);
    controller.setRumble(RumbleType.kRightRumble, 0.0);

    //apply deadband (0.1) and clamp left y and right x between (-1, 1)
    double leftY = MathUtil.clamp(MathUtil.applyDeadband(controller.getLeftY(), 0.1), -1.0, 1.0);
    double rightX = MathUtil.clamp(MathUtil.applyDeadband(controller.getRightX(), 0.1), -1.0, 1.0);

    //add slew rate limiter to x and y and scale motor speed by half
    leftY = yFilter.calculate(Math.abs(leftY) * (leftY) * 0.5);
    rightX = xFilter.calculate(Math.abs(rightX) * (rightX) * 0.5);

    //drive + turn off square inputs
    m_robotDrive.arcadeDrive(leftY, rightX, false);

    //add widgets
    SmartDashboard.putNumber("Right Motor Group", m_rightMotorGroup.get());
    SmartDashboard.putNumber("Left Motor Group", m_leftMotorGroup.get());
    SmartDashboard.putNumber("X slew rate", xFilter.lastValue());
    SmartDashboard.putNumber("Y slew rate", yFilter.lastValue());
  }
}