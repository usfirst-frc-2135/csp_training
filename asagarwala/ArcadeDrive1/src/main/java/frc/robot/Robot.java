// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  
  private final MotorController m_leftRearMotor = new WPI_TalonSRX(1);
  private final MotorController m_leftFrontMotor = new WPI_TalonSRX(2);
  private final MotorController m_rightRearMotor = new WPI_TalonSRX(3);
  private final MotorController m_rightFrontMotor = new WPI_TalonSRX(4);

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
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotorGroup.setInverted(true);
    controller = new XboxController(0):
    controller.setRumble(RumbleType.kLeftRumble, 1.0);
    controller.setRumble(RumbleType.kRightRumble, 1.0);
  
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-controller.getLeftY(), -controller.getRightX());

    SmartDashboard.putData("Left Motor Group", m_leftMotorGroup);
    SmartDashboard.putData("Right Motor Group", m_rightMotorGroup);
    SmartDashboard.putData("LeftY", controller.getLeftY());
    SmartDashboard.putData("RightX", controller.getRightX());

    controller.setRumble(RumbleType.kLeftRumble, 0.0);
    controller.setRumble(RumbleType.kRightRumble, 0.0);
  }
  
}
  
