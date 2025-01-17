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
  // Constants
  
  private final static double               kDt              = 0.020; // Loop delay time for simulation
  private final static int                  kGamepadPort     = 0;     // XBox m_controller USB port
  private final static int                  kMotorCANId      = 5;     // Motor CAN ID assignment
  private final static double               kP               = 0.125; // PID - proportional value
  private final static double               kI               = 0.0;   // PID - integral value
  private final static double               kD               = 0.0;   // PID - derivative value
  private final static double               kEncoderCPR      = 4096;  // Encoder CPR for CTRE Mag encoder connected to Talon SRX
  private final static double               kConstantOutput  = 0.3;   // Constant percent output value
  private final static double               kMaxVelocity     = 1.0;   // Trapezoidal profile max velocity
  private final static double               kMaxAcceleration = 2.0;   // Trapezoidal profile max acceleration
  private final static double               kForwardGoal     = 3.0;   // Trapezoidal move - forward goal rotations
  private final static double               kReverseGoal     = 0.0;   // Trapezoidal move - reverse goal rotations
  private final static double               kGoalTolerance   = 0.01;   // Tolerance around the target goal allowed to consider move is finished (positive OR negative)

  // Member objects
  private final XboxController              m_controller     = new XboxController(kGamepadPort);
  private final ExampleSmartMotorController m_motor          = new ExampleSmartMotorController(kMotorCANId, kEncoderCPR); // TODO: This was missing the second parameter!
  private final TalonSRXSimCollection       m_motorSim       = m_motor.getMotorSimulation( );
  private final ElevSim                     m_elevSim        = new ElevSim(m_motorSim, kEncoderCPR);

  private TrapezoidProfile                  m_trapezoidalProfile;
  private boolean                           m_pidEnabled     = false;

  // private final static double kv = 1.0;
  // private final static double ka = 2.0;
  public void TrapezoidalProfile( )
  {
    final double kv = 1.0;
    final double ka = 2.0;
    TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);
  }



  // Create a motion profile with the given maximum velocity and maximum acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile  = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal     = new TrapezoidProfile.State( );
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State( );

 

  @Override
  public void robotInit( )
  {
    DataLogManager.start( );  // TODO: This enables logging for the entire robot, just needed when robot starts

    m_motor.resetEncoder( );
    m_motor.setPID(kP, kI, kP);
  }

  @Override
  public void robotPeriodic( )
  {
    m_elevSim.periodic( );
  }

  @Override
  public void teleopPeriodic( )
  {
    SmartDashboard.putNumber("elevator_rotations", m_motor.getEncoderRotations( ));
    SmartDashboard.putNumber("Goal", m_goal.position);
    // SmartDashboard.putNumber("Error", )

    if (m_controller.getAButtonPressed( ))
    {
      DataLogManager.log("A button pressed"); 
      m_motor.set(kConstantOutput);
      m_pidEnabled = false;
      
    }

    if (m_controller.getBButtonPressed( ))
    {
      DataLogManager.log("B button pressed"); 
      m_motor.set(-kConstantOutput);
      m_pidEnabled = false;
      
    }

    if (m_controller.getRightBumperPressed( ))
    {
      DataLogManager.log("Right bumper pressed");
      m_motor.stopMotor( );
      m_pidEnabled = false;
    }

    if (m_controller.getStartButtonPressed( ))
    {
      DataLogManager.log("Start button pressed"); 
      m_motor.setInverted(!m_motor.getInverted( ));
      m_pidEnabled = false;
      //
      // TODO: The above line does all of this - have a look at how it does this and then delete
      // if (m_motor.getInverted( ))
      // {
      //   m_motor.setInverted(false);
      // }
      // else
      // {
      //   m_motor.setInverted(true);
      // }
    }

    if (m_controller.getXButtonPressed( ))
    {
      DataLogManager.log("X button pressed"); 
      m_goal = new TrapezoidProfile.State(kForwardGoal, 0.0);
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 1.0, 0.0);
      m_pidEnabled = true;
    }
    if (m_controller.getYButtonPressed( ))
    {
      DataLogManager.log("Y button pressed"); 
      m_goal = new TrapezoidProfile.State(kReverseGoal, 0.0);
      m_motor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, 0.0, 0.0);
      m_pidEnabled = true;
    }
  

  // TODO: here is pseudo code for the PID control loop with a trapezoidal profile
  //
    if (m_pidEnabled)
    {
        m_setpoint = profile.calculate(kDt, m_setpoint, m_goal);
        m_motor.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);
        if ((Math.abs(m_goal.position - m_setpoint.position)) < kGoalTolerance)
        {
            m_pidEnabled = false;
        }
    }
  
  //  for all of the buttons above, either enable the pid (X and Y) or disable the pid (A, B, Right Bumper, Start)
  //    a simple boolean can be used to enable or disable the PID processing
  }

}


// TODO: This can be deleted once the pid pseudo code has been implemented
//
// Retrieve the profiled setpoint for the next timestep. This setpoint moves
// toward the goal while obeying the constraints.
/*
 * m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
 * 
 * // Send setpoint to offboard controller PID
 * m_motor.setSetpoint(
 * ExampleSmartMotorController.PIDMode.kPosition,
 * m_setpoint.position,
 * m_feedforward.calculate(m_setpoint.velocity) / 12.0);
 * }
 */
