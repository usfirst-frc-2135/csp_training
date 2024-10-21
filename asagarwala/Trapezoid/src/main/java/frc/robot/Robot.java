// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class Robot extends TimedRobot {
  private static double kDt = 0.02;
  SlewRateLimiter xfilter = new SlewRateLimiter(2);
  SlewRateLimiter yfilter = new SlewRateLimiter(2);

  private final XboxController m_controller = new XboxController(0);
  private final MotorController m_leftRearMotor = new WPI_TalonSRX(1);
  private final MotorController m_leftFrontMotor = new WPI_TalonSRX(2);
  private final MotorController m_rightRearMotor = new WPI_TalonSRX(3);
  private final MotorController m_rightFrontMotor = new WPI_TalonSRX(4);
  private final MotorController m_TopMotor = new WPI_TalonSRX(5);

  MotorControllerGroup m_leftMotorGroup = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  MotorControllerGroup m_rightMotorGroup = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotorGroup::set, m_rightMotorGroup::set);

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotorGroup);
    SendableRegistry.addChild(m_robotDrive, m_rightMotorGroup);
  }
  //private final ExampleSmartMotorController m_motor = new ExampleSmartMotorController(1);
  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  @Override
  public void robotInit() {
    // Note: These gains are fake, and will have to be tuned for your robot.
    m_rightMotorGroup.setInverted(true);
    //controller = new XboxController(0);
    WPI_TalonSRX TopMotor = (WPI_TalonSRX) m_leftRearMotor;
    TopMotor.config_kP(0, 1.3); // set proportional gain
    leftRearMotor.config_kI(0, 0.0); // set integral gain
    leftRearMotor.config_kD(0, 0.7); // set derivative gain
  
    m_controller.setRumble(RumbleType.kLeftRumble, 1.0);
    m_controller.setRumble(RumbleType.kRightRumble, 1.0);
  }

  @Override
  public void teleopPeriodic() {

    yfilter.calculate(m_controller.getLeftY());
    xfilter.calculate(m_controller.getRightX());
    m_robotDrive.arcadeDrive(yfilter.calculate(m_controller.getLeftY()), xfilter.calculate(m_controller.getRightX()));

    if (m_controller.getRawButtonPressed(2)) {
      m_goal = new TrapezoidProfile.State(5, 0);
    } else if (m_controller.getRawButtonPressed(3)) {
      m_goal = new TrapezoidProfile.State();
    }

    double LeftY = MathUtil.clamp(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1), -1.0, 1.0);
    double RightX = MathUtil.clamp(MathUtil.applyDeadband(m_controller.getRightX(), 0.1), -1.0, 1.0);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    double squareLeft = yfilter.calculate(LeftY * Math.abs(LeftY)*0.5);
    double squareRight = xfilter.calculate(RightX * Math.abs(RightX)*0.5);

    m_robotDrive.arcadeDrive(squareLeft, squareRight, false);

    SmartDashboard.putNumber("Left Motor Group", m_leftMotorGroup.get());
    SmartDashboard.putNumber("Right Motor Group", m_rightMotorGroup.get());
    SmartDashboard.putNumber("Slew X", xfilter.lastValue());
    SmartDashboard.putNumber("Slew Y", yfilter.lastValue());

    m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
    m_controller.setRumble(RumbleType.kRightRumble, 0.0);

    // Send setpoint to offboard controller PID
  WPI_TalonSRX leftRearMotor = (WPI_TalonSRX) m_leftRearMotor;
      leftRearMotor.set(ControlMode.Position, m_setpoint.position, 
         DemandType.ArbitraryFeedForward, m_feedforward.calculate(m_setpoint.velocity) / 12.0);
  }
}
