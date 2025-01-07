// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ExampleSmartMotorController.PIDMode;

public class Robot extends TimedRobot
{
  // Constants (Magic Numbers)
  private final static double                kDt              = 0.020; // Loop delay time for simulation
  private final static int                   kGamepadPort     = 0;     // XBox m_controller USB port
  private final static int                   kMotorCANId      = 5;     // Motor CAN ID assignment
  private final static double                kP               = 0.38;  // PID - proportional value
  private final static double                kI               = 0.0;   // PID - integral value
  private final static double                kD               = 0.0;   // PID - derivative value
  private final static double                kEncoderCPR      = 4096;  // Encoder CPR for CTRE Mag encoder connected to Talon SRX
  private final static double                kConstantOutput  = 0.3;   // Constant percent output value
  private final static double                kMaxVelocity     = 1.0;   // Trapezoidal profile max velocity
  private final static double                kMaxAcceleration = 2.0;   // Trapezoidal profile max acceleration
  private final static double                kForwardGoal     = 3.0;   // Trapezoidal move - forward goal rotations
  private final static double                kReverseGoal     = 0.0;   // Trapezoidal move - reverse goal rotations
  private final static double                kGoalTolerance   = 0.01;   // Tolerance around the target goal allowed to consider move is finished (positive OR negative)

  // Member objects
  private final XboxController               m_controller     = new XboxController(kGamepadPort);
  private final ExampleSmartMotorController  m_motor          = new ExampleSmartMotorController(kMotorCANId, kEncoderCPR);
  private final TalonSRXSimCollection        m_motorSim       = m_motor.getMotorSimulation( );
  private final ElevSim                      m_elevSim        = new ElevSim(m_motorSim, kEncoderCPR);

  // Create a motion profile with the given maximum velocity and maximum acceleration constraints for the next setpoint.
  private final TrapezoidProfile.Constraints m_constraints    = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final TrapezoidProfile             m_profile        = new TrapezoidProfile(m_constraints);
  private TrapezoidProfile.State             m_setpoint       = new TrapezoidProfile.State( );
  private TrapezoidProfile.State             m_goal           = new TrapezoidProfile.State(kReverseGoal, 0.0);
  private boolean                            m_pidEnabled     = false;


  public void robotInit( )
  {
    DataLogManager.start( ); 
    m_motor.setPID(kP, kI, kD);
    m_motor.resetEncoder( );
    m_elevSim.reset( );

    SmartDashboard.putNumber("Kp", m_motor.getKp( ));  
  }

  @Override
  public void disabledInit( )
  {
    m_pidEnabled = false;
    m_motor.stopMotor( );
  }

  @Override
  public void robotPeriodic( )
  {
    m_elevSim.periodic( );
  }

  @Override
  public void teleopPeriodic( )
  {
    SmartDashboard.putNumber("Goal", m_goal.position);
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));
    SmartDashboard.putNumber("elevator rotations", m_motor.getEncoderRotations( ));

    if (m_controller.getAButtonPressed( ))
    {
      DataLogManager.log("A button pressed");
      m_pidEnabled = false;
      m_motor.setOutput(kConstantOutput);
    }

    if (m_controller.getBButtonPressed( ))
    {
      DataLogManager.log("B button pressed");
      m_pidEnabled = false;
      m_motor.setOutput(-kConstantOutput);
    }

    if (m_controller.getRawButtonPressed(8))
    {
      DataLogManager.log("Start button pressed");
      m_pidEnabled = false;
      m_motor.setInverted(!m_motor.getInverted( ));
    }

    if (m_controller.getRightBumperPressed( ))
    {
      DataLogManager.log("Right bumper pressed");
      m_pidEnabled = false;
      m_motor.stopMotor( );
    }

    if (m_controller.getXButtonPressed( ))
    {
      DataLogManager.log("X button pressed");
      m_goal = new TrapezoidProfile.State(kForwardGoal, 0);    // Change the goal
      m_pidEnabled = true;
      DataLogManager.log("New m_goal " + m_goal.position);
    }

    if (m_controller.getYButtonPressed( ))
    {
      DataLogManager.log("Y button pressed");
      m_goal = new TrapezoidProfile.State(kReverseGoal, 0);    // Change the goal
      m_pidEnabled = true;
      DataLogManager.log("New m_goal " + m_goal.position);
    }

    SmartDashboard.putBoolean("pidEnabled", m_pidEnabled);

    if (m_pidEnabled)
    {
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);  // Get a new setpoint by passing in the current setpoint and goal
      m_motor.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);

      if ((Math.abs(m_goal.position - m_setpoint.position)) < kGoalTolerance)
      {
        m_pidEnabled = false;
      }
    }

  }
}
