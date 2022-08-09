package teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="practicebot", group="Linear Opmode")
@Disabled

public class practicebot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor bl = null;
    private DcMotor br = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        tl = hardwareMap.get(DcMotor.class, "tl");
        tr = hardwareMap.get(DcMotor.class, "tr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        tl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        tr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

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
            double normalmode = 1;

            tl.setPower(normalmode*(turn + (vertical) + horizontal));
            tr.setPower(normalmode*((-turn) + (vertical) - horizontal));
            bl.setPower(normalmode*(turn + (vertical) - horizontal));
            br.setPower(normalmode*((-turn) + (vertical) + horizontal));

            if (gamepad1.a) {
                tl.setPower(1);
                tr.setPower(1);
                bl.setPower(1);
                br.setPower(1);
            }
            if(gamepad1.b) {
                claw.setPosition(1);
            }
        }
    }
}

//new arm down