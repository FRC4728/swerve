package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.DRIVER;
import frc.robot.Constants.HARDWARE;

public class Drivetrain {
    
    private final SwervePod mFrontLeft =
        new SwervePod(  "FrontLeft", 
                        HARDWARE.FRONT_LEFT_DRIVE_MOTOR_ID, 
                        HARDWARE.FRONT_LEFT_TURN_MOTOR_ID,
                        HARDWARE.FRONT_LEFT_QUAD_A_DIO_CHANNEL,
                        HARDWARE.FRONT_LEFT_QUAD_B_DIO_CHANNEL, 
                        HARDWARE.FRONT_LEFT_PWM_DIO_CHANNEL,
                        false,
                        true,
                        DRIVETRAIN.FRONT_LEFT_ZERO_RAD ); 

    private final SwervePod mRearLeft =
        new SwervePod(  "RearLeft",
                        HARDWARE.REAR_LEFT_DRIVE_MOTOR_ID,
                        HARDWARE.REAR_LEFT_TURN_MOTOR_ID,
                        HARDWARE.REAR_LEFT_QUAD_A_DIO_CHANNEL,
                        HARDWARE.REAR_LEFT_QUAD_B_DIO_CHANNEL,
                        HARDWARE.REAR_LEFT_PWM_DIO_CHANNEL,
                        false,
                        true,
                        DRIVETRAIN.REAR_LEFT_ZERO_RAD );

    private final SwervePod mFrontRight =
        new SwervePod(  "FrontRight",
                        HARDWARE.FRONT_RIGHT_DRIVE_MOTOR_ID,
                        HARDWARE.FRONT_RIGHT_TURN_MOTOR_ID,
                        HARDWARE.FRONT_RIGHT_QUAD_A_DIO_CHANNEL,
                        HARDWARE.FRONT_RIGHT_QUAD_B_DIO_CHANNEL,
                        HARDWARE.FRONT_RIGHT_PWM_DIO_CHANNEL,
                        true,
                        true,
                        DRIVETRAIN.FRONT_RIGHT_ZERO_RAD );

    private final SwervePod mRearRight =
        new SwervePod(  "RearRight",
                        HARDWARE.REAR_RIGHT_DRIVE_MOTOR_ID,
                        HARDWARE.REAR_RIGHT_TURN_MOTOR_ID,
                        HARDWARE.REAR_RIGHT_QUAD_A_DIO_CHANNEL,
                        HARDWARE.REAR_RIGHT_QUAD_B_DIO_CHANNEL,
                        HARDWARE.REAR_RIGHT_PWM_DIO_CHANNEL,
                        true,
                        true,
                        DRIVETRAIN.REAR_RIGHT_ZERO_RAD );

    private final Translation2d mFrontLeftLocation = 
        new Translation2d( Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                           Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 );
    private final Translation2d mFrontRightLocation = 
        new Translation2d( Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                           -Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 );
    private final Translation2d mRearLeftLocation = 
        new Translation2d( -Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                           Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 );
    private final Translation2d mRearRightLocation = 
       new Translation2d( -Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                          -Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 );

    private final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics( mFrontLeftLocation, mFrontRightLocation,
                                   mRearLeftLocation, mRearRightLocation);

    private final Gyro mGyro = new ADXRS450_Gyro();

    private SwerveDriveOdometry mOdometry =
        new SwerveDriveOdometry( kDriveKinematics, mGyro.getRotation2d() );

    private boolean IsHomingFinshed;


    //-------------------------------------------------------------------------------------------//
    /*                                      PUBLIC METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * Get the status of the swerve pods homing.
     * @return True when all pods have finished homing, false otherwise.
     */
    public boolean IsHomingFinshed () {
        return IsHomingFinshed;
    }

    /**
     * This function will forcibly stop all swerve pod homing routines and
     * report the status of each. If this method is called, it means that the
     * homing routine needs to tuned up to finished within the alloted time.
     */
    public void SetIsHomingFinshed () {
        IsHomingFinshed = true;
        DriverStation.reportWarning( "Swerve homing timeout: FrontLeft: "+mFrontLeft.IsHomingFinshed()+
                                                         ", FrontRight: "+mFrontRight.IsHomingFinshed()+                                                 
                                                           ", RearLeft: "+mRearLeft.IsHomingFinshed()+                                                       
                                                          ", RearRight: "+mRearRight.IsHomingFinshed(), false);
    }

    /**
     * Start the homing routine of the swerve pods.
     */
    public void StartHomingSwervePods () {
        mFrontLeft.StartHoming();
        mFrontRight.StartHoming();
        mRearLeft.StartHoming();
        mRearRight.StartHoming();

    }

    /**
     * Update the homing routine of the swerve pods. If all of the pods have
     * finished, then set IsHomingFinshed. In the case were some pods have
     * finished and some have not, it doesn't hurt to call a homing update
     * on the finished pods.
     */
    public void HomingSwervePodsUpdate () {
        if ( !IsHomingFinshed ) {
            if ( mFrontLeft.IsHomingFinshed() && 
                 mFrontRight.IsHomingFinshed() &&
                 mRearLeft.IsHomingFinshed() &&  
                 mRearRight.IsHomingFinshed() ) {
                IsHomingFinshed = true;
            } else {
                mFrontLeft.HomingUpdate();
                mFrontRight.HomingUpdate();
                mRearLeft.HomingUpdate();
                mRearRight.HomingUpdate();
            }
        }
    }

    /**
     * Teleop interface to drive the robot.
     * 
     * TODO: The field relative driving requires an accurate measure of the
     * robots heading. Need to come up with a process to ensure accurate
     * heading measurements througout the match (like when to do heading
     * resets, and etc.).
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Rate of the robot rotation.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     * the field or the robot.
     */
    public void Drive (double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, mGyro.getRotation2d() )
                          : new ChassisSpeeds( xSpeed, ySpeed, rot ) );
    
        // Clamp the wheel speeds to what the driver/robot are capable of
        // handling.
        SwerveDriveKinematics.normalizeWheelSpeeds( swerveModuleStates, DRIVER.MAX_DRIVE_VELOCITY_MPS );

        mFrontLeft.SetDesiredState( swerveModuleStates[0] );
        mFrontRight.SetDesiredState( swerveModuleStates[1] );
        mRearLeft.SetDesiredState( swerveModuleStates[2] );
        mRearRight.SetDesiredState( swerveModuleStates[3] );
    }


    //-------------------------------------------------------------------------------------------//
    /*                                     PRIVATE METHODS                                       */
    //-------------------------------------------------------------------------------------------//


    /**
     * This function will make periodic updates to the state of the swerve
     * pods. The order of the pods MUST match the order of the constructor
     * of the SwerveDriveKinematics.
     */  
    private void UpdateOdometry () {
        mOdometry.update( mGyro.getRotation2d(), 
                          mFrontLeft.GetState(),
                          mFrontRight.GetState(), 
                          mRearLeft.GetState(), 
                          mRearRight.GetState());
    }

    /** 
     * Reset the robots heading to 0.
     * */
    private void ResetHeading () {
        mGyro.reset();
    }

    // /**
    //  * Get the robots heading.
    //  *
    //  * @return the robots heading in degrees (-180 to 180)
    //  */
    // private double GetHeading() {
    //     return mGyro.getRotation2d().getDegrees();
    // }

    // /**
    //  * Get the robots rate of turn.
    //  *
    //  * @return The turn rate of the robot (degrees per second)
    //  */
    // private double GetTurnRate() {
    //     return mGyro.getRate();
    // }


    //-------------------------------------------------------------------------------------------//
    /*                            CONSTRUCTOR AND PERIODIC METHODS                               */
    //-------------------------------------------------------------------------------------------//


    /**
     * The constructor for the Drivetrain class.
     */  
    public Drivetrain () {
        IsHomingFinshed = false;
        ResetHeading();
    }

    /**
     * This function will make the periodic updates and thus should be called
     * periodically.
     */  
    public void PeriodicUpdate () {
        mFrontLeft.PeriodicUpdate();
        mFrontRight.PeriodicUpdate();
        mRearLeft.PeriodicUpdate();
        mRearRight.PeriodicUpdate();
        UpdateOdometry();
    }


}
