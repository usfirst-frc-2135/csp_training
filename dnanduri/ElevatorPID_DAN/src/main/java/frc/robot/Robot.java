// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;

public class Robot extends TimedRobot
{
  private final static double                kDt           = 0.020;

  // TODO:  these define a trapezoidal profile with a max velocity of 8.0 rps and acceleration of 16 rps/sec
  //      further down, m_profile is created with a much slower profile of 1.75 rps and 0.75 rps/sec
  //      and then after that m_constraints is defined with the faster setings
  //      these are not used YET, but be aware that you have multiple (confilicting) profiles defined
  private final static double                kv            = 8.0; // Max velocity - RPS
  private final static double                ka            = 16.0;
  private double                             goal;
  //private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);

  private final XboxController               controller    = new XboxController(0);
  private final ExampleSmartMotorController  m_motor       = new ExampleSmartMotorController(5);

  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward       m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile             m_profile     = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));

  private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(kv, ka);

  private TrapezoidProfile.State             m_goal        = new TrapezoidProfile.State( );   // The desired end state of the movement
  private TrapezoidProfile.State             m_setpoint    = new TrapezoidProfile.State( );   // The currently active state while making the movement

  @Override
  public void robotInit( )
  {
    // Note: These gains are fake, and will have to be tuned for your robot.
    m_motor.setPID(0.125, 0.0, 0.0);
    DataLogManager.start( );

    DataLogManager.log("Initial encoder position: " + m_motor.getEncoderDistance( ));
  }

  @Override
  public void teleopPeriodic( )
  {
    SmartDashboard.putNumber("Goal", goal);
    SmartDashboard.putNumber("Kp", m_motor.getKp( ));   // TODO: since the motor kp cannot be changed while running, this doesn't need to be in the periodic loop (put in robotInit after setPID)
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));

    SmartDashboard.putNumber("elevator rotations", m_motor.getEncoderDistance( ));
    if (controller.getAButtonPressed( ))
    {
      m_motor.set(0.3);
      DataLogManager.log("A button pressed"); // TODO: These log messages help greatly during debugging, why not do ALL the buttons while learning (X, Y, 8 are omitted)
    }
    else if (controller.getBButtonPressed( ))
    {
      m_motor.set(-0.3);
      DataLogManager.log("B button pressed");
    }

    if (controller.getRawButtonPressed(8))  // TODO: I'll bet there's a named get<something>Pressed that is a better call than this
    {
      if (m_motor.getInverted( ))
      {
        m_motor.setInverted(false);
        DataLogManager.log("Motor Inverted = false");
      }
      else
      {
        m_motor.setInverted(true);
        DataLogManager.log("Motor Inverted = true");
      }
    }

    if (controller.getXButtonPressed( ))
    {
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 1.0, 0.0);
      goal = 4096.0;  // TODO: the goal (endpoint) will be in rotations (not counts), so it should be 1.0 and could be passed into the setSetpoint method above FOR THE PID MOVE
    }

    if (controller.getYButtonPressed( ))
    {
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 0.0, 0.0);
      goal = 0.0; // TODO: the goal (endpoint) will be in rotations (not counts), so it should be 1.0 and could be passed into the setSetpoint method above FOR THE PID MOVE
    }

    if (controller.getRightBumperPressed( ))
    {
      m_motor.stopMotor( );
      DataLogManager.log("Right bumper pressed, motor stopped");
    }

    //var profile = new TrapezoidProfile(m_Constraints, m_goal, m_setpoint);
    //var profile = new TrapezoidProfile(m_Constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    //m_setpoint = m_profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    // m_motor.setSetpoint(
    //     ExampleSmartMotorController.PIDMode.kPosition,
    //     m_setpoint.position,
    //     m_feedforward.calculate(m_setpoint.velocity) / 12.0);
  }
}
