package frc.robot.subsystems;


import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants.DRIVETRAIN;
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
                        DRIVETRAIN.FRONT_LEFT_ZERO_RAD ); 
    private final SwervePod mRearLeft =
        new SwervePod(  "RearLeft",
                        HARDWARE.REAR_LEFT_DRIVE_MOTOR_ID,
                        HARDWARE.REAR_LEFT_TURN_MOTOR_ID,
                        HARDWARE.REAR_LEFT_QUAD_A_DIO_CHANNEL,
                        HARDWARE.REAR_LEFT_QUAD_B_DIO_CHANNEL,
                        HARDWARE.REAR_LEFT_PWM_DIO_CHANNEL,
                        false,
                        DRIVETRAIN.REAR_LEFT_ZERO_RAD );
    private final SwervePod mFrontRight =
        new SwervePod(  "FrontRight",
                        HARDWARE.FRONT_RIGHT_DRIVE_MOTOR_ID,
                        HARDWARE.FRONT_RIGHT_TURN_MOTOR_ID,
                        HARDWARE.FRONT_RIGHT_QUAD_A_DIO_CHANNEL,
                        HARDWARE.FRONT_RIGHT_QUAD_B_DIO_CHANNEL,
                        HARDWARE.FRONT_RIGHT_PWM_DIO_CHANNEL,
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
                        DRIVETRAIN.REAR_RIGHT_ZERO_RAD );
    private final Gyro mGyro = new ADXRS450_Gyro();
    private final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(  Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                                Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 ),
            new Translation2d(  Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                                -Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 ),
            new Translation2d(  -Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                                Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 ),
            new Translation2d(  -Units.inchesToMeters( DRIVETRAIN.WHEEL_BASE_INCH ) / 2,
                                -Units.inchesToMeters( DRIVETRAIN.TRACK_WIDTH_INCH ) / 2 ) );
    private SwerveDriveOdometry mOdometry =
        new SwerveDriveOdometry( kDriveKinematics, mGyro.getRotation2d() );
        private boolean mIsHomingFinshed;

    /**
     * Output telemetry to the network tables.
     */  
    public void OutputTelemetry () {
        mFrontLeft.OutputTelemetry();
        mRearLeft.OutputTelemetry();
        mFrontRight.OutputTelemetry();
        mRearRight.OutputTelemetry();
    }


    /**
     *  This function will make periodic updates.
     */  
    public void PeriodicUpdate () {
        mOdometry.update( mGyro.getRotation2d(), mFrontLeft.GetState(), mRearLeft.GetState(), mFrontRight.GetState(), mRearRight.GetState());
    }


    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d GetPose() {
        return mOdometry.getPoseMeters();
    }


    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void ResetOdometry ( Pose2d pose ) {
        mOdometry.resetPosition( pose, mGyro.getRotation2d() );
    }


    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void Drive (double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, mGyro.getRotation2d() )
                          : new ChassisSpeeds( xSpeed, ySpeed, rot ) );
        SwerveDriveKinematics.normalizeWheelSpeeds( swerveModuleStates, Units.feetToMeters( DRIVETRAIN.MAX_DRIVE_VELOCITY_FPS ) );

        mFrontLeft.SetDesiredState( swerveModuleStates[0] );
        mFrontRight.SetDesiredState( swerveModuleStates[1] );
        mRearLeft.SetDesiredState( swerveModuleStates[2] );
        mRearRight.SetDesiredState( swerveModuleStates[3] );
    }
    



    public void StartingHomingPods () {
        mFrontLeft.StartHoming();
        mRearLeft.StartHoming();
        mFrontRight.StartHoming();
        mFrontRight.StartHoming();

    }


    public void HomingPodsUpdate () {
        if ( !mIsHomingFinshed ) {
            if ( mFrontLeft.GetIsHomingFinshed() && mRearLeft.GetIsHomingFinshed() && mFrontRight.GetIsHomingFinshed() && mFrontRight.GetIsHomingFinshed() ) {
                mIsHomingFinshed = true;
                mFrontLeft.ResetEncoders();
                mRearLeft.ResetEncoders();
                mFrontRight.ResetEncoders();
                mFrontRight.ResetEncoders();
            } else {
                mFrontLeft.HomingUpdate();
                mRearLeft.HomingUpdate();
                mFrontRight.HomingUpdate();
                mFrontRight.HomingUpdate();
            }
        }
    }

    
    public boolean GetIsHomingFinshed () {
        return mIsHomingFinshed;
    }





    /** 
     * Resets the drive encoders to currently read a position of 0.
     */
    public void ResetEncoders() {
        mFrontLeft.ResetEncoders();
        mRearLeft.ResetEncoders();
        mFrontRight.ResetEncoders();
        mRearRight.ResetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void ZeroHeading() {
        mGyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double GetHeading() {
        return mGyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double GetTurnRate() {
        return mGyro.getRate();
    }

    /**
    * The constructor for the Drivetrain class.
    */  
    public Drivetrain () {
        mIsHomingFinshed = false;
    }

}
