// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{
  public static double                      kDt           = 0.02;
  private double                            goal;
  private final static double               kEncoderCPR   = 4096;

  private final XboxController              m_controller  = new XboxController(0);
  private final ExampleSmartMotorController m_motor       = new ExampleSmartMotorController(5, kEncoderCPR);
  private final SimpleMotorFeedforward      m_feedforward = new SimpleMotorFeedforward(1, 1.5);
  private final TrapezoidProfile            m_profile     = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.0, 2.0));
  private TrapezoidProfile.State            m_goal        = new TrapezoidProfile.State( );
  private TrapezoidProfile.State            m_setpoint    = new TrapezoidProfile.State( );

  private final TalonSRXSimCollection       m_motorSim    = m_motor.getMotorSimulation( );
  private final ElevSim                     m_elevSim     = new ElevSim(m_motorSim, kEncoderCPR);

  @Override
  public void robotInit( )
  {
    // Note: These gains are fake, and will have to be tuned for your robot.
    m_motor.setPID(0.27, 0.0, 0.0);
    m_motor.resetEncoder( );
    m_elevSim.periodic( );
    DataLogManager.start( );
  }

  @Override
  public void teleopPeriodic( )
  {
    /*
     * if (m_joystick.getRawButtonPressed(2)) {
     * m_goal = new TrapezoidProfile.State(5, 0);
     * } else if (m_joystick.getRawButtonPressed(3)) {
     * m_goal = new TrapezoidProfile.State();
     * }
     * 
     * // Retrieve the profiled setpoint for the next timestep. This setpoint moves
     * // toward the goal while obeying the constraints.
     * m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
     * 
     * // Send setpoint to offboard controller PID
     * m_motor.setSetpoint(
     * ExampleSmartMotorController.PIDMode.kPosition,
     * m_setpoint.position,
     * m_feedforward.calculate(m_setpoint.velocity) / 12.0);
     */

    // SmartDashboard.putNumber("Elevator Rotations", m_motor.getEncoderDistance());
    // SmartDashboard.putNumber("Target", goal);
    // SmartDashboard.putNumber("Error", m_motor.getClosedLoopError());

    SmartDashboard.putNumber("Elevator Rotations", m_motor.getEncoderDistance( ));
    SmartDashboard.putNumber("Target", goal);
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));
    SmartDashboard.putNumber("Kp", m_motor.getKp( ));
    SmartDashboard.putNumber("Velocity", m_motor.getVelocity( ));

    m_elevSim.periodic( );

    if (m_controller.getAButtonPressed( ))
    { // if A button pressed, set voltage of 0.3
      m_motor.set(0.3);
      DataLogManager.log("A Button Pressed -- Voltage PercentOutput: 0.3");
    }

    if (m_controller.getBButtonPressed( ))
    { // if B button pressed, set voltage of -0.3
      m_motor.set(-0.3);
      DataLogManager.log("B Button Pressed -- Voltage PercentOutput: -0.3");
    }

    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
    m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position,
        m_feedforward.calculate(m_setpoint.velocity) / 12.0); // why divide by 12?

    if (m_controller.getXButtonPressed( ))
    { // if X button pressed, set PID at setpoint of 1.0
      // m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 1.0, 0); 
      // DataLogManager.log("X Button Pressed -- PID Setpoint: 1.0");
      // goal = 4096.0;
      m_goal = new TrapezoidProfile.State(5, 0); // test this value (position value :)
      DataLogManager.log("X Button Pressed -- Trapezoid Profile Setpoint: 1.0");
    }

    if (m_controller.getYButtonPressed( ))
    { // if Y button pressed, set PID at setpoint of 0.0
      // m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 0.0, 0);
      // DataLogManager.log("Y Button Pressed -- PID Setpoint: 0.0");
      // goal = 0.0;
      m_goal = new TrapezoidProfile.State(0, 0); // test this value (position value :)
      DataLogManager.log("Y Button Pressed -- Trapezoid Profile Setpoint: 0.0");
    }

    if (m_controller.getRightBumperPressed( ))
    { // if Right Bumper pressed, stop Motor
      m_motor.stopMotor( );
      DataLogManager.log("Right Bumper Pressed -- Motor Stopped");
    }

    if (m_controller.getRawButtonPressed(8))
    { // if menu button, invert motor direction
      DataLogManager.log("Menu Button Pressed"); // controller is not reading the button being pressed?
      if (m_motor.getInverted( ))
      {
        m_motor.setInverted(false);
        DataLogManager.log("Motor Inverted -- False");
      }
      else
      {
        m_motor.setInverted(true);
        DataLogManager.log("Motor Inverted -- True");
      }
    }
  }
}
