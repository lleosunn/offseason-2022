package teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleop", group="Linear Opmode")

public class teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor intake = null;
    private DcMotor arm = null;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        tl = hardwareMap.get(DcMotor.class, "tl");
        tr = hardwareMap.get(DcMotor.class, "tr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        Servo lever = hardwareMap.get(Servo.class, "lever");
        CRServo spinner = hardwareMap.get(CRServo.class, "spinner");

        tl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        tr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double vertical;
            double horizontal;
            double turn;
            double armpower;
            vertical = gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            armpower = gamepad2.left_stick_y;
            double slowmode = .3;

            if (gamepad1.right_trigger > 0) { //slowmode
                tl.setPower(slowmode*(turn + (vertical) + horizontal));
                tr.setPower(slowmode*((-turn) + (vertical) - horizontal));
                bl.setPower(slowmode*(turn + (vertical) - horizontal));
                br.setPower(slowmode*((-turn) + (vertical) + horizontal));
            }
            else { //normal
                tl.setPower(1*(turn + (vertical) + horizontal));
                tr.setPower(1*((-turn) + (vertical) - horizontal));
                bl.setPower(1*(turn + (vertical) - horizontal));
                br.setPower(1*((-turn) + (vertical) + horizontal));
            }

            if (gamepad2.right_trigger > 0) { // intake
                intake.setPower(-1);
            } else intake.setPower(0);
            if (gamepad2.left_trigger > 0) { //outtake
                intake.setPower(1);
            } else intake.setPower(0);

            arm.setPower(-armpower); //arm

            if (gamepad2.a) {
                lever.setPosition(0.5); //release
            }
            if (gamepad2.b) {
                lever.setPosition(1); //reset
            }
            if (gamepad2.y) {
                lever.setPosition(0.85); //transfer position
            }

            if (gamepad2.dpad_left) { //duck spinner
                spinner.setPower(1);
            } else spinner.setPower(0);
            if (gamepad2.dpad_right) {
                spinner.setPower(-1);
            } else spinner.setPower(0);

            if (gamepad2.x) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad2.left_stick_y < 0 || gamepad2.left_stick_y > 0) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-armpower);
            }

        }
    }
}

//new arm down