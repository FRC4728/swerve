package frc.robot.utils;

public class SwerveDriveSignal {

    private final double mXSpeed_mps;
    private final double mYSpeed_mps;
    private final double mRotation_rps;
    private final boolean mFieldOriented;

    public double GetXSpeed () {
        return mXSpeed_mps;
    }

    public double GetYSpeed () {
        return mYSpeed_mps;
    }

    public double GetRotation() {
        return mRotation_rps;
    }

    public boolean GetIsFieldOriented() {
        return mFieldOriented;
    }

    /**
     * 
     * @param xSpeed
     * @param ySpeed
     * @param rotation
     * @param fieldOriented
     */
    public SwerveDriveSignal (double xSpeed, double ySpeed, double rotation, boolean fieldOriented) {
        mXSpeed_mps = xSpeed;
        mYSpeed_mps = ySpeed;
        mRotation_rps = rotation;
        mFieldOriented = fieldOriented;
    }


}
