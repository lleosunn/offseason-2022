package teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
public class oldRWH extends LinearOpMode {

    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor intake = null;
    private DcMotor arm = null;
    private DistanceSensor left;
    private DistanceSensor right;
    private DistanceSensor back;
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
            tfod.setZoom(1, 16.0/9.0); //sets camera zoom and aspect ratio
        }

        tl = hardwareMap.get(DcMotor.class, "tl");
        tr = hardwareMap.get(DcMotor.class, "tr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        Servo lever = hardwareMap.get(Servo.class, "lever");
        CRServo spinner = hardwareMap.get(CRServo.class, "spinner");
        left = hardwareMap.get(DistanceSensor.class, "left");
        right = hardwareMap.get(DistanceSensor.class, "right");
        back = hardwareMap.get(DistanceSensor.class, "back");
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) left;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) right;
        Rev2mDistanceSensor sensorTimeOfFlight3 = (Rev2mDistanceSensor) back;
        intake.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        motorblock block = new motorblock(tl, tr, bl, br);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("angle", getAngle());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            resetStartTime();
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lever.setPosition(1);
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
                        //updatedRecognitions.get(0).getLabel().equals("Duck");
                        telemetry.update();
                        if (updatedRecognitions.get(0).getLeft() < 220){ //left
                            while (time < 1) {
                                block.leftturn(0.025 * (getAngle() - 25));
                            }
                            block.stop();
                            block.backward(0.38);
                            arm.setTargetPosition(5500);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(1);
                            sleep(700);
                            block.stop();
                            sleep(1800);
                            lever.setPosition(0.5);
                            sleep(1000);
                            lever.setPosition(1);
                            arm.setTargetPosition(0);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(0.8);
                            sleep(3000);
                            while (time < 12) {
                                block.leftturn(0.02 * (getAngle() - 90));
                            }
                            resetAngle();
                            sleep(500);
                            while (time < 14) {
                                double power = 0.2;
                                double differenceSide = (right.getDistance(DistanceUnit.INCH) - 0.25);
                                double side = differenceSide*0.3;
                                double tlSide = (side*(-(power + (0.01 * getAngle()))));
                                double trSide = (side*(-(power + (0.01 * getAngle()))));
                                double blSide = (side*((power - (0.01 * getAngle()))));
                                double brSide = (side*((power - (0.01 * getAngle()))));

                                tl.setPower(tlSide);
                                tr.setPower(trSide);
                                bl.setPower(blSide);
                                br.setPower(brSide);

                                if (-0.5 < differenceSide && differenceSide < 0.5) {
                                    block.leftturn(0.035*getAngle());
                                }
                            }
                            block.stop();
                            block.forward(0.3);
                            sleep(4000);
                            block.stop();
                            stop();
                        }
                        else { //middle
                            while (time < 1) {
                                block.leftturn(0.025 * (getAngle() - 25));
                            }
                            block.stop();
                            block.backward(0.3);
                            arm.setTargetPosition(4700);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(1);
                            sleep(1100);
                            block.stop();
                            sleep(1800);
                            lever.setPosition(0.5);
                            sleep(1000);
                            lever.setPosition(1);
                            arm.setTargetPosition(0);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(0.8);
                            sleep(3000);
                            while (time < 12) {
                                block.leftturn(0.02 * (getAngle() - 90));
                            }
                            resetAngle();
                            sleep(500);
                            while (time < 14) {
                                double power = 0.2;
                                double differenceSide = (right.getDistance(DistanceUnit.INCH) - 0.25);
                                double side = differenceSide*0.3;
                                double tlSide = (side*(-(power + (0.01 * getAngle()))));
                                double trSide = (side*(-(power + (0.01 * getAngle()))));
                                double blSide = (side*((power - (0.01 * getAngle()))));
                                double brSide = (side*((power - (0.01 * getAngle()))));

                                tl.setPower(tlSide);
                                tr.setPower(trSide);
                                bl.setPower(blSide);
                                br.setPower(brSide);

                                if (-0.5 < differenceSide && differenceSide < 0.5) {
                                    block.leftturn(0.035*getAngle());
                                }
                            }
                            block.stop();
                            block.forward(0.3);
                            sleep(4000);
                            block.stop();
                            stop();
                        }
                    } else { //right
                        while (time < 1) {
                            block.leftturn(0.025 * (getAngle() - 25));
                        }
                        block.stop();
                        block.backward(0.3);
                        arm.setTargetPosition(3300);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(1);
                        sleep(2000);
                        block.stop();
                        sleep(1800);
                        lever.setPosition(0.5);
                        sleep(1000);
                        lever.setPosition(1);
                        arm.setTargetPosition(0);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.8);
                        sleep(3000);
                        block.forward(0.3);
                        sleep(500);
                        block.stop();
                        sleep(500);
                        while (time < 15) {
                            block.leftturn(0.02 * (getAngle() - 90));
                        }
                        resetAngle();
                        sleep(500);
                        while (time < 17) {
                            double power = 0.2;
                            double differenceSide = (right.getDistance(DistanceUnit.INCH) - 0.25);
                            double side = differenceSide*0.3;
                            double tlSide = (side*(-(power + (0.01 * getAngle()))));
                            double trSide = (side*(-(power + (0.01 * getAngle()))));
                            double blSide = (side*((power - (0.01 * getAngle()))));
                            double brSide = (side*((power - (0.01 * getAngle()))));

                            tl.setPower(tlSide);
                            tr.setPower(trSide);
                            bl.setPower(blSide);
                            br.setPower(brSide);

                            if (-0.5 < differenceSide && differenceSide < 0.5) {
                                block.leftturn(0.035*getAngle());
                            }
                        }
                        block.stop();
                        sleep(1000);
                        block.forward(0.3);
                        sleep(4000);
                        block.stop();
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
