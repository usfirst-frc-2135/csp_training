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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.ActuatorIn;
// import frc.robot.commands.ActuatorOut;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // public final Servo m_actuator = new Servo(0);
  public final RobotContainer m_robotContainer = new RobotContainer(); // Create that robot

  private final CommandXboxController m_controller = new CommandXboxController(0);

  // private final Servo m_actuator = new Servo(0);

  private final MotorController m_leftRearMotor = new WPI_TalonSRX(1);
  private final MotorController m_leftFrontMotor = new WPI_TalonSRX(2);
  private final MotorController m_rightRearMotor = new WPI_TalonSRX(3);
  private final MotorController m_rightFrontMotor = new WPI_TalonSRX(4);

  MotorControllerGroup m_leftMotorGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  MotorControllerGroup m_rightMotorGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotorGroup::set, m_rightMotorGroup::set);

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotorGroup);
    SendableRegistry.addChild(m_robotDrive, m_rightMotorGroup);
    configureButtonBindings();

    // AddDashboardWidgets();

  }

  // @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // l_actuator.setBoundsMicroseconds(1000000, 900000, 500000, 100000, 0);

    m_rightMotorGroup.setInverted(true);

    m_controller.setRumble(RumbleType.kLeftRumble, 1.0);
    m_controller.setRumble(RumbleType.kRightRumble, 1.0);

    // l_actuator.set(0.1);
    // l_actuator.set(0.75);
    // l_actuator.setBounds(maxPulseWidth, centerPulseWidth, minPulseWidth))

    // m_actuator.set(0.25);

  }

  private void configureButtonBindings() {
    // m_controller.a(l_actuator.set(0.1));
    // m_controller.b().onTrue(Commands.runOnce(() -> l_actuator.set(0.25)));

  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopPeriodic() {

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    m_robotDrive.arcadeDrive(-m_controller.getLeftY() * 0.713, -m_controller.getRightX() * 0.713);

    m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
    m_controller.setRumble(RumbleType.kRightRumble, 0.0);
  }

}
