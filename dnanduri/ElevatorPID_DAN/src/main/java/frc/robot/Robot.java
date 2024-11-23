// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;

public class Robot extends TimedRobot {

  private final static double kDt = 0.020;

  private final static double kv = 1.0; // Max velocity - RPS
  private final static double ka = 2.0;
  private double goal;
  private final static double kEncoderCPR = 4096;
  // private final static TrapezoidProfile.Constraints m_constraints = new
  // TrapezoidProfile.Constraints(kv, ka);

  private final XboxController controller = new XboxController(0);
  private final static ExampleSmartMotorController m_motor = new ExampleSmartMotorController(5);

  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));

  private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(kv, ka);

  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  public class ElevSim {
    private final static TalonSRXSimCollection m_motorSim = m_motor.getMotorSimulation();
    // private final static ElevSim m_elevSim = new ElevSim(m_motorSim,
    // kEncoderCPR);
  }

  @Override
  public void robotInit() {
    // Note: These gains are fake, and will have to be tuned for your robot.
    m_motor.setPID(0.125, 0.0, 0.0);
    m_motor.resetEncoder();

    DataLogManager.start();

    DataLogManager.log("Initial encoder position: " + m_motor.getEncoderDistance());

  }

  @Override
  public void teleopPeriodic() {
    // m_elevSim.periodic();
    SmartDashboard.putNumber("Goal", goal);
    SmartDashboard.putNumber("Kp", m_motor.getKp());
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError());

    SmartDashboard.putNumber("elevator rotations", m_motor.getEncoderDistance());
    if (controller.getAButtonPressed()) {
      m_motor.set(0.3);
      DataLogManager.log("A button pressed");
    } else if (controller.getBButtonPressed()) {
      m_motor.set(-0.3);
      DataLogManager.log("B button pressed");
    }

    if (controller.getRawButtonPressed(8)) {
      if (m_motor.getInverted()) {
        m_motor.setInverted(false);
        DataLogManager.log("Motor Inverted = false");
      } else {
        m_motor.setInverted(true);
        DataLogManager.log("Motor Inverted = true");
      }
    }

    if (controller.getXButtonPressed()) {
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 1.0, 0.0);
      goal = 4096.0;
    }

    if (controller.getYButtonPressed()) {
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 0.0, 0.0);
      goal = 0.0;
    }

    if (controller.getRightBumperPressed()) {
      m_motor.stopMotor();
      DataLogManager.log("Right bumper pressed, motor stopped");
    }

    // var profile = new TrapezoidProfile(m_Constraints, m_goal, m_setpoint);
    // var profile = new TrapezoidProfile(m_Constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    // m_setpoint = m_profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    // m_motor.setSetpoint(
    // ExampleSmartMotorController.PIDMode.kPosition,
    // m_setpoint.position,
    // m_feedforward.calculate(m_setpoint.velocity) / 12.0);
  }
}
