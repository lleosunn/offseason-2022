package teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleop2", group="Linear Opmode")

public class teleop2 extends LinearOpMode {

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

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double vertical;
            double horizontal;
            double turn;
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            double slowmode = .1;
            double fastmode = 1;
            double normalmode = 0.6;

            if (gamepad1.right_trigger > 0) { //slowmode
                tl.setPower(slowmode*(turn + (vertical) + horizontal));
                tr.setPower(slowmode*((-turn) + (vertical) - horizontal));
                bl.setPower(slowmode*(turn + (vertical) - horizontal));
                br.setPower(slowmode*((-turn) + (vertical) + horizontal));
            }
            else { //normal
                tl.setPower(normalmode*(turn + (vertical) + horizontal));
                tr.setPower(normalmode*((-turn) + (vertical) - horizontal));
                bl.setPower(normalmode*(turn + (vertical) - horizontal));
                br.setPower(normalmode*((-turn) + (vertical) + horizontal));
            }

            if (gamepad1.left_trigger > 0) { //fastmode
                tl.setPower(fastmode*(turn + (vertical) + horizontal));
                tr.setPower(fastmode*((-turn) + (vertical) - horizontal));
                bl.setPower(fastmode*(turn + (vertical) - horizontal));
                br.setPower(fastmode*((-turn) + (vertical) + horizontal));
            }
            else { //normal
                tl.setPower(normalmode*(turn + (vertical) + horizontal));
                tr.setPower(normalmode*((-turn) + (vertical) - horizontal));
                bl.setPower(normalmode*(turn + (vertical) - horizontal));
                br.setPower(normalmode*((-turn) + (vertical) + horizontal));
            }

            if (gamepad2.right_trigger > 0) { // intake
                intake.setPower(1);
            }
            else if (gamepad2.left_trigger > 0) { //outtake
                intake.setPower(-1);
            }
            else intake.setPower(0);

            if (gamepad2.y) {
                lift.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                lift2.setPower(1);
            }
            if (gamepad2.right_bumper) {
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
                lift2.setPower(1);
            }
            if (gamepad2.dpad_up) { //lift up
                lift.setTargetPosition(3500);
                lift2.setTargetPosition(-3500);
            }
            if (gamepad2.dpad_down) { //lift down
                lift.setTargetPosition(0);
                lift2.setTargetPosition(0);
            }
            while (gamepad2.left_stick_y < 0 || gamepad2.left_stick_y > 0) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad2.left_stick_y < 0) {
                    lift.setPower(0.5);
                    lift2.setPower(-0.5);
                }
                else if (gamepad2.left_stick_y > 0) {
                    lift.setPower(-0.5);
                    lift2.setPower(0.5);
                }
                else {
                    lift.setPower(0);
                    lift2.setPower(0);
                }

            }

            if (gamepad2.b) {
                lever.setPosition(0.96);
                lever2.setPosition(0.04);
            }
            if (gamepad2.a) {
                lever.setPosition(.30);
                lever2.setPosition(0.72);
            }
            if(gamepad2.x) {
                lever.setPosition(0.66);
                lever2.setPosition(0.35);
            }
            if (gamepad2.dpad_left) {
                intake.setPower(-0.75);
            } else spinner.setPower(0);
            if (gamepad2.dpad_right) {
                intake.setPower(0.75);
            } else spinner.setPower(0);

        }
    }
}

//new arm down