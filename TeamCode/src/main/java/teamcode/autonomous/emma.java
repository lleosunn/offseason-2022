package teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import teamcode.motorblock;

import java.util.List;

@Autonomous
public class emma extends LinearOpMode {
    //declaring motors, distance sensors, timer
    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor intake = null;
    private DcMotor lift = null;
    private DcMotor lift2 = null;
    private DistanceSensor left;
    private DistanceSensor right;
    private DistanceSensor front;
    private ElapsedTime runtime = new ElapsedTime();

    //declare gyro (direction, angle)
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    //initialize gyro
    public void imuinit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        //send data to controller (screen), see angle while coding
        telemetry.addData("Gyro Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Gyro Mode", "ready");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }
    //when changes direction resets angle to 0
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    //run program
    @Override
    public void runOpMode() {
        imuinit();

        //quotations --> what put in calibration file, assigns port to motor (ex. poto 9 - "tl")
        tl = hardwareMap.get(DcMotor.class, "tl");
        tr = hardwareMap.get(DcMotor.class, "tr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        Servo lever = hardwareMap.get(Servo.class, "lever");
        Servo lever2 = hardwareMap.get(Servo.class, "lever2");
        right = hardwareMap.get(DistanceSensor.class, "right");
        left = hardwareMap.get(DistanceSensor.class, "left");
        front = hardwareMap.get(DistanceSensor.class, "front");


        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) left;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) right;
        Rev2mDistanceSensor sensorTimeOfFlight3 = (Rev2mDistanceSensor) front;
        intake.setDirection(DcMotor.Direction.FORWARD);
        motorblock block = new motorblock(tl, tr, bl, br);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("angle", getAngle());
        telemetry.update();

        resetAngle();

        waitForStart();

        //when it starts --> self-correcting gyro
        while (opModeIsActive()) {
            //set angle to 0
            //robot moves, angle changes
            //reset angle back to 0, using power and angle

            //double angle_correction = 0.02 * getAngle();
           // double distance_correction = -0.03 * (front.getDistance(DistanceUnit.INCH) - 12);



            block.leftturn(0.02 * getAngle());
            //setting power to angle
            //tl.setPower(-angle_correction + distance_correction);
            //tr.setPower(-angle_correction - distance_correction);
            //bl.setPower(-angle_correction + distance_correction);
           // br.setPower(-angle_correction - distance_correction);
            //initialize 12 inches away from wall, when move robot closer or farther from 12 inches, readjusts back. has distance sensor
            // block.forward(-0.06 * (front.getDistance(DistanceUnit.INCH) - 12));
            /*
            tl.setPower(power);
            tr.setPower(-power);
            bl.setPower(power);
            br.setPower(-power);
             */
            //combine gyro and distance sensor



        }
    }
}
