package teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.motorblock;

import java.util.List;

@TeleOp(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")

public class camera extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor intake = null;
    private DcMotor lift = null;
    private DcMotor lift2 = null;
    private DcMotor spinner = null;

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
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        Servo lever = hardwareMap.get(Servo.class, "lever");
        Servo lever2 = hardwareMap.get(Servo.class, "lever2");

        tl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        tr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        motorblock block = new motorblock(tl, tr, bl, br);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
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
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
