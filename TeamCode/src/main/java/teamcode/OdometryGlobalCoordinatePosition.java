package teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometryGlobalCoordinatePosition {
    //Odometry wheels
    private DcMotor encoderLeft, encoderRight, encoderAux;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double currentRightPos = 0, currentLeftPos = 0, currentAuxPos = 0, currentIMU = 0,  orientationChange = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double oldRightPos = 0, oldLeftPos = 0, oldAuxPos = 0, oldIMU = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public OdometryGlobalCoordinatePosition(DcMotor encoderLeft, DcMotor encoderRight, DcMotor encoderAux){
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderAux = encoderAux;

    }

    public void globalCoordinatePositionUpdate(double orientation){
        oldLeftPos = currentLeftPos;
        oldRightPos = currentRightPos;
        oldAuxPos = currentAuxPos;
        oldIMU = currentIMU;

        currentLeftPos = -encoderLeft.getCurrentPosition();
        currentRightPos = encoderRight.getCurrentPosition();
        currentAuxPos = -encoderAux.getCurrentPosition();
        currentIMU = orientation;

        double leftChange = currentLeftPos - oldLeftPos;
        double rightChange = currentRightPos - oldRightPos;
        double auxChange = currentAuxPos - oldAuxPos;
        double IMUChange = currentIMU - oldIMU;

        orientationChange = (rightChange - leftChange) / 27703.414;
//        orientationChange = Math.toRadians(IMUChange);
//        robotOrientationRadians = Math.toRadians(orientation); //using imu
        robotOrientationRadians = ((robotOrientationRadians + orientationChange));

        double horizontalChange = auxChange - (orientationChange * 8937.34946515); //9250

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (n*Math.cos(robotOrientationRadians) - p*Math.sin(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (n*Math.sin(robotOrientationRadians) + p*Math.cos(robotOrientationRadians));
    }

    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians); }

    public void stop(){ isRunning = false; }

}
