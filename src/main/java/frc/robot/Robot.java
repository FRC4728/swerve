package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.HARDWARE;
import frc.robot.Constants.DRIVER;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;

public class Robot extends TimedRobot {

    private final Drivetrain mDrivetrain = new Drivetrain();
    private final Joystick mDriverRightJoystick = new Joystick( HARDWARE.RIGHT_JOYSTICK );
    private final Joystick mDriverLeftJoystick = new Joystick( HARDWARE.LEFT_JOYSTICK );
    private final SlewRateLimiter mXSpeedLimiter = new SlewRateLimiter( DRIVER.DRIVE_X_SLEW_RATE );
    private final SlewRateLimiter mYSpeedLimiter = new SlewRateLimiter( DRIVER.DRIVE_Y_SLEW_RATE );
    private final SlewRateLimiter mRotLimiter = new SlewRateLimiter( DRIVER.DRIVE_ROT_SLEW_RATE );
    private Timer mHomingTimer = new Timer();


    // TODO: This function will be available in the 2022 WPIlib, use it
    private double applyDeadband ( double value, double deadband ) {
        if ( Math.abs( value ) > deadband ) {
            if ( value > 0.0 ) {
                return (value - deadband) / ( 1.0 - deadband );
            } else {
                return ( value + deadband ) / ( 1.0 - deadband );
            }
        } else {
            return 0.0;
        }
    }



    @Override
    public void robotInit() {
        // Use the live window telemetry during bringup...don't want to waste
        // time with it otherwise.
        LiveWindow.disableAllTelemetry();
    }

    
    @Override
    public void robotPeriodic() {
        mDrivetrain.PeriodicUpdate();
    }
    
    @Override
    public void autonomousInit() {}
    

    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopInit() {
        mDrivetrain.CalibrateGyro();
        // TODO: Currently, the homing will only execute the first time teleop
        // is run after powering up. In other words, is teleop is disabled and
        // restarted, homing will not be run. Need a homing plan that satisfies
        // real gameplay and practice.
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
                // This will print out a warning message and the status of the swerve pods
                mDrivetrain.SetIsHomingFinshed(); 
            }

        } //else {
            // // These signs are setup for my APEM joysticks
            // final var xSpeed = mXSpeedLimiter.calculate( applyDeadband( mDriverRightJoystick.getX(), 0.02 ) ) * DRIVER.MAX_DRIVE_VELOCITY_MPS;
            // final var ySpeed = -mYSpeedLimiter.calculate( applyDeadband( mDriverRightJoystick.getY(), 0.02 ) ) * DRIVER.MAX_DRIVE_VELOCITY_MPS;
            // final var rotSpeed = mRotLimiter.calculate( applyDeadband( mDriverRightJoystick.getZ(), 0.02 ) ) * DRIVER.MAX_DRIVE_ANGULAR_VELOCITY_RPS;
            // mDrivetrain.Drive(xSpeed, ySpeed, rotSpeed, true);
        //}
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
