// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private XboxController controller;

  private final MotorController m_leftRearMotor = new WPI_TalonSRX(1);
  private final MotorController m_leftFrontMotor = new WPI_TalonSRX(2);
  private final MotorController m_rightRearMotor = new WPI_TalonSRX(3);
  private final MotorController m_rightFrontMotor = new WPI_TalonSRX(4);

  @Override
  public void robotInit() {
    /*
    SendableRegistry.addChild(m_robotDrive, );
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    */
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    MotorControllerGroup m_leftMotorGroup = new MotorControllerGroup(m_leftFrontMotor,m_leftRearMotor);
    MotorControllerGroup m_rightMotorGroup = new MotorControllerGroup(m_rightFrontMotor,m_rightRearMotor);

    m_rightMotorGroup.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftMotorGroup::set, m_rightMotorGroup::set);
    controller = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(controller.getLeftY(), controller.getRightY());
  }
}