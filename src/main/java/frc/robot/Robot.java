package frc.robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


public class Robot extends TimedRobot {

    private final Drivetrain mDrivetrain = new Drivetrain();
    
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
