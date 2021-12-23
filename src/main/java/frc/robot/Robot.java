package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.HARDWARE;

/**
 * The VM is configured to automatically run this class, and to call the 
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final Encoder mQuadEncoder1 = new Encoder( HARDWARE.SWERVE_POD_1_QUAD_A_DIO_CHANNEL, HARDWARE.SWERVE_POD_1_QUAD_B_DIO_CHANNEL, false, CounterBase.EncodingType.k4X);
    private final DutyCycleEncoder mPWMEncoder1 = new DutyCycleEncoder( HARDWARE.SWERVE_POD_1_PWM_DIO_CHANNEL );

    @Override
    public void robotInit() {
      mPWMEncoder1.setDistancePerRotation( 360 );                               // A full cycle is 360deg
      mQuadEncoder1.setSamplesToAverage( 5 );                                   // Angular velocity calculation
      mQuadEncoder1.setDistancePerPulse( 1.0 / 1024.0 * 360.0);                 // Pulse to degrees
      mQuadEncoder1.setMinRate(1.0);                                            // Rate at which mechanism is considered "stopped"

    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putBoolean( "/PWM1/Connected", mPWMEncoder1.isConnected() );
        SmartDashboard.putNumber( "/PWM1/Frequency", mPWMEncoder1.getFrequency() );
        SmartDashboard.putNumber( "/PWM1/Output", mPWMEncoder1.get() );
        SmartDashboard.putNumber( "/PWM1/Distance", mPWMEncoder1.getDistance() );
        SmartDashboard.putNumber( "/Quad1/Distance", mQuadEncoder1.getDistance() );
        SmartDashboard.putNumber( "/Quad1/Rate", mQuadEncoder1.getRate() );

    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
