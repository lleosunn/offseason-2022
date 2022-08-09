package teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="solo", group="Linear Opmode")

public class solo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor intake = null;
    private DcMotor lift = null;
    private DcMotor lift2 = null;
    private DcMotor spinner = null;

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


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        tl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        tr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        tl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        imuinit();
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double theta = Math.toRadians(getAngle());

            double tlH = x * Math.sin(theta - (Math.PI/4));
            double trH = x * Math.cos(theta - (Math.PI/4));
            double blH = x * Math.cos(theta - (Math.PI/4));
            double brH = x * Math.sin(theta - (Math.PI/4));

            double tlV = y * Math.sin(theta + (Math.PI/4));
            double trV = y * Math.cos(theta + (Math.PI/4));
            double blV = y * Math.cos(theta + (Math.PI/4));
            double brV = y * Math.sin(theta + (Math.PI/4));

//            tl.setPower(y + x - turn);
//            tr.setPower(y - x + turn);
//            bl.setPower(y - x - turn);
//            br.setPower(y + x + turn);

            tl.setPower(tlV - tlH - turn);
            tr.setPower(trV - trH + turn);
            bl.setPower(blV - blH - turn);
            br.setPower(brV - brH + turn);

            if (gamepad1.right_stick_button) {
                resetAngle();
            }

            if (gamepad1.right_trigger > 0) { // intake
                intake.setPower(1);
            } else intake.setPower(0);

            if (gamepad1.left_trigger > 0) { //outtake
                intake.setPower(-1);
            } else intake.setPower(0);

            if (gamepad1.y) {
                lift.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                lift2.setPower(1);
            }
            if (gamepad1.right_bumper) {
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                lift2.setPower(1);
            }
            if (gamepad1.dpad_up) { //lift up
                lift.setTargetPosition(3500);
                lift2.setTargetPosition(-3500);
            }
            if (gamepad1.dpad_down) { //lift down
                lift.setTargetPosition(0);
                lift2.setTargetPosition(0);
            }

            if (gamepad1.b) {
                lever.setPosition(0.96);
                lever2.setPosition(0.04);
            }
            if (gamepad1.a) {
                lever.setPosition(.30);
                lever2.setPosition(0.72);
            }
            if(gamepad1.x) {
                lever.setPosition(0.66);
                lever2.setPosition(0.35);
            }
            if (gamepad1.dpad_left) {
                intake.setPower(-0.75);
            } else spinner.setPower(0);
            if (gamepad1.dpad_right) {
                intake.setPower(0.75);
            } else spinner.setPower(0);

        }
    }
}

//new arm down