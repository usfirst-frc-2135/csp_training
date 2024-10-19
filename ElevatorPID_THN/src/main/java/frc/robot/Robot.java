// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private static double kDt = 0.02;

  private final XboxController m_controller = new XboxController(0);
  private final ExampleSmartMotorController m_motor = new ExampleSmartMotorController(1);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  @Override
  public void robotInit() {
    // Note: These gains are fake, and will have to be tuned for your robot.
    m_motor.setPID(0.5, 0.0, 0.0);
    DataLogManager.start();
  }

  @Override
  public void teleopPeriodic() {
  /*
    if (m_joystick.getRawButtonPressed(2)) {
      m_goal = new TrapezoidProfile.State(5, 0);
    } else if (m_joystick.getRawButtonPressed(3)) {
      m_goal = new TrapezoidProfile.State();
    }

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    // Send setpoint to offboard controller PID
    m_motor.setSetpoint(
        ExampleSmartMotorController.PIDMode.kPosition,
        m_setpoint.position,
        m_feedforward.calculate(m_setpoint.velocity) / 12.0);
    */

    SmartDashboard.putNumber("Elevator Rotations", m_motor.getEncoderDistance());

    if (m_controller.getAButtonPressed()) { // if A button pressed, set PID at voltage of 0.3
      m_motor.set(0.3);
      DataLogManager.log("A Button Pressed -- Voltage PercentOutput: 0.3");
    }

    if (m_controller.getBButtonPressed()) { // if B button pressed, set PID at voltage of -0.3
      m_motor.set(-0.3);
      DataLogManager.log("B Button Pressed -- Voltage PercentOutput: -0.3");
    }

    if (m_controller.getRightBumperPressed()) { // if Right Bumper pressed, stop Motor
      m_motor.stopMotor();
      DataLogManager.log("Right Bumper Pressed -- Motor Stopped");
    }

    if (m_controller.getRawButtonPressed(8)) { // if menu button, invert motor direction
      DataLogManager.log("Menu Button Pressed"); // controller is not reading the button being pressed?
      if (m_motor.getInverted()) {
        m_motor.setInverted(false);
        DataLogManager.log("Motor Inverted -- False");
      } else {
        m_motor.setInverted(true);
        DataLogManager.log("Motor Inverted -- True");
      }
    }
  }
}
