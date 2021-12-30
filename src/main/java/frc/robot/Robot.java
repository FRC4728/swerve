package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CONTROL;

public class Robot extends TimedRobot {

    private RobotContainer mRobotContainer = new RobotContainer();
    private UpdateManager mUpdateManager = new UpdateManager(
        mRobotContainer.GetDrivetrainSubsystem()
    );

    @Override
    public void robotInit() {
        // Use the live window telemetry during bringup...don't want to waste
        // time with it otherwise.
        LiveWindow.disableAllTelemetry();
        mUpdateManager.StartLoop( CONTROL.LOOP_TIME_S );
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void autonomousInit() {}
    

    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopInit() {
        mRobotContainer.GetDrivetrainSubsystem().HomeModules();
    }
    

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
