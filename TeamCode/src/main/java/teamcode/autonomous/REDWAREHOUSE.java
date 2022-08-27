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
public class REDWAREHOUSE extends LinearOpMode {

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


    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    public void imuinit() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

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
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",

    };
    private static final String VUFORIA_KEY =
            "ASK1IFv/////AAABmfX8FNhgmkD+sKsolRwQ6ShIKlMBrgagR3WWfuzHquZIi3lQ5TFj8sAK1gmeujmQm8I62YVaT/Z3X5XIHGawMeaNm7BnVeU5scz+HdlNLlVrCDbuIb8sJ29tn/mBfWuv3Hvy40iP9uOtEAKi9diyxRsAiytHqfsCvccIem7+C7O6Zbiz3awD5CobXCqevYjWdUGGZarPM2eAyL/NfCswBIdYd1Hjb3VHqJ/tyb1VZdWYlme0pkAKLWbYpRKFlq5Q8EOrzmVWTAw4fCLziKSKYFVFFPNoibGm7cAGsqk+5UD3kOpcoPOhSlDM3GEPMYartSro28s1DB1uFaojEVNE+PJ5fMaWWCsfi9japsdcrP6Q";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        imuinit();
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0); //sets camera zoom and aspect ratio
        }

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

        waitForStart();

        while (opModeIsActive()) {

            lever2.setPosition(0.35);
            resetStartTime();
            telemetry.addData("angle", getAngle());
            telemetry.update();

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;

                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                    telemetry.update();


                    if (updatedRecognitions.isEmpty() != true) {
                        telemetry.update();
                        if (updatedRecognitions.get(0).getLeft() < 220) { //left
                            while (time < 2) {
                                block.leftturn(0.015 * (getAngle() - 35));
                            }
                            block.stop();

                            block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            block.setDrivetrainTarget(950, -950, 950, -950);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                            block.setDrivetrainPower(0.5, 0.5, 0.5, 0.5);
                            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lift.setTargetPosition(500);
                            lift2.setTargetPosition(-500);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                            lift2.setPower(1);
                            sleep(3000);
                            block.stop();

                            lever2.setPosition(0.72);
                            sleep(500);
                            lever2.setPosition(0.35);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            //forward a bit 5.5 seconds
                            block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            block.setDrivetrainTarget(-250, 250, -250, 250);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                            block.setDrivetrainPower(0.3, 0.3, 0.3, 0.3);
                            sleep(2000);
                            block.stop();
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            //straighten robot 10 seconds
                            while (time < 9) {
                                block.leftturn(0.02 * (getAngle() - 90));
                            }
                            resetAngle();

                            //strafe right to wall 13 seconds
                            while (time < 12) {
                                block.rightDistance(0.2, (right.getDistance(DistanceUnit.INCH) - 3),
                                        (right.getDistance(DistanceUnit.INCH) - 3) * 0.4, getAngle());
                                lift.setTargetPosition(0);
                                lift2.setTargetPosition(0);
                                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                lift.setPower(1);
                                lift2.setPower(1);
                            }
                            //park
                            while (time < 16) {
                                block.frontDistance(0.2, (front.getDistance(DistanceUnit.INCH) - 30),
                                        (front.getDistance(DistanceUnit.INCH) - 30) * 0.2, getAngle());
                            }
                            block.stop();
                            block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            block.setDrivetrainTarget(1500, 1500, -1500, -1500);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                            block.setDrivetrainPower(0.3, 0.3, 0.3, 0.3);
                            sleep(5000);
                            stop();
                        } else { //middle
                            while (time < 2) {
                                block.leftturn(0.015 * (getAngle() - 35));
                            }
                            block.stop();

                            block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            block.setDrivetrainTarget(1000, -1000, 1000, -1000);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                            block.setDrivetrainPower(0.5, 0.5, 0.5, 0.5);
                            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lift.setTargetPosition(1800);
                            lift2.setTargetPosition(-1800);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                            lift2.setPower(1);
                            sleep(3000);
                            block.stop();

                            lever2.setPosition(0.72);
                            sleep(500);
                            lever2.setPosition(0.35);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            //forward a bit 5.5 seconds
                            block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            block.setDrivetrainTarget(-300, 300, -300, 300);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                            block.setDrivetrainPower(0.3, 0.3, 0.3, 0.3);
                            sleep(2000);
                            block.stop();
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            //straighten robot 10 seconds
                            while (time < 9) {
                                block.leftturn(0.02 * (getAngle() - 90));
                            }
                            resetAngle();

                            //strafe right to wall 13 seconds
                            while (time < 12) {
                                block.rightDistance(0.2, (right.getDistance(DistanceUnit.INCH) - 3),
                                        (right.getDistance(DistanceUnit.INCH) - 3) * 0.4, getAngle());
                                lift.setTargetPosition(0);
                                lift2.setTargetPosition(0);
                                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                lift.setPower(1);
                                lift2.setPower(1);
                            }
                            //park
                            while (time < 16) {
                                block.frontDistance(0.2, (front.getDistance(DistanceUnit.INCH) - 30),
                                        (front.getDistance(DistanceUnit.INCH) - 30) * 0.2, getAngle());
                            }
                            block.stop();
                            block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            block.setDrivetrainTarget(1500, 1500, -1500, -1500);
                            block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                            block.setDrivetrainPower(0.3, 0.3, 0.3, 0.3);
                            sleep(5000);
                            stop();
                        }
                    } else { //right
                        while (time < 2) {
                            block.leftturn(0.015 * (getAngle() - 35));
                        }
                        block.stop();

                        block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        block.setDrivetrainTarget(1100, -1100, 1100, -1100);
                        block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                        block.setDrivetrainPower(0.5, 0.5, 0.5, 0.5);
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift.setTargetPosition(3200);
                        lift2.setTargetPosition(-3200);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                        lift2.setPower(1);
                        sleep(3000);
                        block.stop();

                        lever2.setPosition(0.72);
                        sleep(500);
                        lever2.setPosition(0.35);
                        block.setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        //forward a bit 5.5 seconds
                        block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        block.setDrivetrainTarget(-400, 400, -400, 400);
                        block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                        block.setDrivetrainPower(0.3, 0.3, 0.3, 0.3);
                        sleep(2000);
                        block.stop();
                        block.setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        //straighten robot 10 seconds
                        while (time < 9) {
                            block.leftturn(0.02 * (getAngle() - 90));
                        }
                        resetAngle();

                        //strafe right to wall 13 seconds
                        while (time < 12) {
                            block.rightDistance(0.2, (right.getDistance(DistanceUnit.INCH) - 3),
                                    (right.getDistance(DistanceUnit.INCH) - 3) * 0.4, getAngle());
                            lift.setTargetPosition(0);
                            lift2.setTargetPosition(0);
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            lift.setPower(1);
                            lift2.setPower(1);
                        }
                        //park
                        while (time < 16) {
                            block.frontDistance(0.2, (front.getDistance(DistanceUnit.INCH) - 30),
                                    (front.getDistance(DistanceUnit.INCH) - 30) * 0.2, getAngle());
                        }
                        block.stop();
                        block.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        block.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        block.setDrivetrainTarget(1500, 1500, -1500, -1500);
                        block.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
                        block.setDrivetrainPower(0.3, 0.3, 0.3, 0.3);
                        sleep(5000);
                        stop();
                    }
                }
            }
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
