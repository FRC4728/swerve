package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DRIVER;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;

public class Robot extends TimedRobot {

    private final Drivetrain mDrivetrain = new Drivetrain();
    //private final Joystick mDriverRightJoystick = new Joystick( DRIVER.RIGHT_JOYSTICK );
    private final Joystick mDriverLeftJoystick = new Joystick( DRIVER.LEFT_JOYSTICK );
    private final SlewRateLimiter mXSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter mYSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter mRotLimiter = new SlewRateLimiter(3);    

    private final double kMaxTurningSpeed_rps = 2.0;
    private Timer mHomingTimer = new Timer();

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
    }



    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
    }
    
    @Override
    public void robotPeriodic() {
        mDrivetrain.OutputTelemetry();
    }
    
    @Override
    public void autonomousInit() {}
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {
        if ( !mDrivetrain.IsHomingFinshed() ) {
            mHomingTimer.start();
            mDrivetrain.StartHomingSwervePods();
        }
    }
    
    @Override
    public void teleopPeriodic() {
        if ( !mDrivetrain.IsHomingFinshed() ) {
            mDrivetrain.HomingSwervePodsUpdate();
            if ( mHomingTimer.hasElapsed( 0.5 ) ) {
                mHomingTimer.stop();
                mDrivetrain.SetIsHomingFinshed();
            }

        } else {
            final var xSpeed = mXSpeedLimiter.calculate( applyDeadband( mDriverLeftJoystick.getX(), 0.02 ) ) * DRIVETRAIN.MAX_DRIVE_VELOCITY_MPS;
            final var ySpeed = mYSpeedLimiter.calculate( applyDeadband( mDriverLeftJoystick.getY(), 0.02 ) ) * DRIVETRAIN.MAX_DRIVE_VELOCITY_MPS;
            final var rotSpeed = mRotLimiter.calculate( applyDeadband( mDriverLeftJoystick.getZ(), 0.02 ) ) * kMaxTurningSpeed_rps;
            mDrivetrain.Drive(xSpeed, ySpeed, rotSpeed, false);
        }
    }
    
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void testInit() {}
    
    @Override
    public void testPeriodic() {}
    
} 
