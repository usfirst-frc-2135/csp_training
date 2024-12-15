// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ExampleSmartMotorController.PIDMode;

public class Robot extends TimedRobot
{
  // Constants
  // TODO: Note that we try to make all numbers use names (these are called literals) to make them describe the value
  //    Use these literals to replace the "magic" numbers in your code--it should improve readability
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
  private final ExampleSmartMotorController  m_motor          = new ExampleSmartMotorController(kMotorCANId, kEncoderCPR); // TODO: This was missing the second parameter!
  private final TalonSRXSimCollection        m_motorSim       = m_motor.getMotorSimulation( );
  private final ElevSim                      m_elevSim        = new ElevSim(m_motorSim, kEncoderCPR);

  // Create a motion profile with the given maximum velocity and maximum acceleration constraints for the next setpoint.
  private final TrapezoidProfile.Constraints m_constraints    = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final TrapezoidProfile             m_profile        = new TrapezoidProfile(m_constraints);
  private TrapezoidProfile.State             m_setpoint       = new TrapezoidProfile.State( );
  private TrapezoidProfile.State             m_goal           = new TrapezoidProfile.State(kReverseGoal, 0.0);
  private Timer                              m_timer          = new Timer( );
  private boolean                            m_pidEnabled     = false;

  // TODO: Not quite sure how inserting this elevator sim class even works here, but you've already created the m_elevSim object on the previous line,
  //        so we can just use it's methods without having all this extraneous sfuff in here.
  // public class ElevSim
  // {
  // ....

  public void robotInit( )
  {
    DataLogManager.start( );  // TODO: Put this first when robot starts so all the following methods can use the logger.

    // m_profile = new TrapezoidProfile(constraints); // TODO: Delete this, it's already done during initialization of class member variables above

    m_motor.setPID(kP, kI, kD);
    m_motor.resetEncoder( );
    m_elevSim.reset( );

    SmartDashboard.putNumber("Kp", m_motor.getKp( ));   // TODO: This can never be changed once the robot is running--I moved it into robotInit() (could be deleted!)
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

  @Override // TODO: Somehow this was removed and teleopPeriodic would never run!
  public void teleopPeriodic( )
  {
    SmartDashboard.putNumber("Goal", m_goal.position);
    SmartDashboard.putNumber("Error", m_motor.getClosedLoopError( ));
    SmartDashboard.putNumber("elevator rotations", m_motor.getEncoderRotations( ));

    if (m_controller.getAButtonPressed( ))
    {
      DataLogManager.log("A button pressed"); // TODO: Put all the button log messages BEFORE the action, so they are chronological in the printed log
      m_pidEnabled = false;
      m_motor.setOutput(kConstantOutput);
    }

    if (m_controller.getBButtonPressed( ))
    {
      DataLogManager.log("B button pressed");
      m_pidEnabled = false;
      m_motor.setOutput(-kConstantOutput);
    }

    if (m_controller.getStartButtonPressed( ))  // TODO: Button 8 is the START button. 
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
      m_timer.restart( );                                               // Start the profile timer
      m_goal = new TrapezoidProfile.State(kForwardGoal, 0);    // Change the goal
      m_pidEnabled = true;
      DataLogManager.log("New m_goal " + m_goal.position);
    }

    if (m_controller.getYButtonPressed( ))
    {
      DataLogManager.log("Y button pressed");
      m_timer.restart( );                                               // Start the profile timer
      m_goal = new TrapezoidProfile.State(kReverseGoal, 0);    // Change the goal
      m_pidEnabled = true;
      DataLogManager.log("New m_goal " + m_goal.position);
    }

    SmartDashboard.putBoolean("pidEnabled", m_pidEnabled);

    if (m_pidEnabled)
    {
      m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);  // Get a new setpoint by passing in the current setpoint and goal
      m_motor.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0); // Adjust feedforward TODO: Arbitrary feedforward is ZERO for this

      if ((Math.abs(m_goal.position - m_setpoint.position)) < kGoalTolerance)
      {
        m_pidEnabled = false;
      }
    }

  }

  // TODO: Providing "getter" functions at this level adds no useful value. No one is calling these--nor should they. 
  //        All data should be passed through the method calls--not using these. I recommend removing them.
  //   public static double getKdt( )
  //   {
  //     return kDt;
  //   }

  //   public static double getKv( )
  //   {
  //     return kv;
  //   }

  //   public static double getKa( )
  //   {
  //     return ka;
  //   }

  //   public double getGoal( )
  //   {
  //     return goal;
  //   }

  //   public void setGoal(double goal)
  //   {
  //     this.goal = goal;
  //   }

  //   public static double getKencodercpr( )
  //   {
  //     return kEncoderCPR;
  //   }

  //   public XboxController getController( )
  //   {
  //     return m_controller;
  //   }

  //   public static ExampleSmartMotorController getmMotor( )
  //   {
  //     return m_motor;
  //   }

  //   public SimpleMotorFeedforward getM_feedforward( )
  //   {
  //     return m_feedforward;
  //   }

  //   public TrapezoidProfile getM_profile( )
  //   {
  //     return m_profile;
  //   }

  //   public TrapezoidProfile.Constraints getM_Constraints( )
  //   {
  //     return m_Constraints;
  //   }

  //   public TrapezoidProfile.State getM_setpoint( )
  //   {
  //     return m_setpoint;
  //   }

  //   public void setM_setpoint(TrapezoidProfile.State m_setpoint)
  //   {
  //     this.m_setpoint = m_setpoint;
  //   }

  //   public TalonSRXSimCollection getM_motorSim( )
  //   {
  //     return m_motorSim;
  //   }

  //   public ElevSim getM_elevSim( )
  //   {
  //     return m_elevSim;
  //   }
}
