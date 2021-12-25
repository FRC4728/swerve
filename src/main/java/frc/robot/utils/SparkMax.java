package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANError;

/**
* This class provides methods for configuring the SparkMax motor controllers.
* @see {@link http://www.revrobotics.com/content/sw/max/sw-docs/java/index.html}
*/
public class SparkMax {
    /**
    * This class provides a method to initialize the motor controller to a
    * known configuration. Also, any sticky faults present will be logged and
    * cleared.
    * <p>
    * <ul>
    * <li>Sticky faults will be logged and cleared
    * <li>Reset to factory defaults
    * <li>Set the idle mode to brake
    * <li>Open-loop duty-cycle output is set to 0.0
    * </ul>
    * <p>
    * @param sparkMax CANSparkMax The motor controller to initialize
    */        
    public static void SetDefaultConfig ( CANSparkMax sparkMax ) {
        CANError canError;
        long faults;

        faults = sparkMax.getStickyFaults();
        if ( faults != 0 ) {
            DriverStation.reportWarning( "Clearing SparkMax "+sparkMax.getDeviceId()+" sticky faults: "+faults, false );
            canError = sparkMax.clearFaults();
            if ( canError != CANError.kOk ) {
                DriverStation.reportWarning( "Could not clear sticky faults due to EC: "+canError.toString(), false );
            }  
        }
        canError = sparkMax.restoreFactoryDefaults();
        if ( canError != CANError.kOk ) {
            DriverStation.reportWarning( "Could not factory reset SparkMax "+sparkMax.getDeviceId()+" due to EC: "+canError.toString(), false );
        }  
        canError = sparkMax.setIdleMode( IdleMode.kBrake );
        if ( canError != CANError.kOk ) {
            DriverStation.reportWarning( "Could not set SparkMax "+sparkMax.getDeviceId()+" idle mode due to EC: "+canError.toString(), false );
        }        
    }

}
