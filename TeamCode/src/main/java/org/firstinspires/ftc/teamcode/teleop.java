package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

            if (gamepad1.right_trigger > 0) {
                tl.setPower(slowmode*(turn + (vertical) + horizontal));
                tr.setPower(slowmode*((-turn) + (vertical) - horizontal));
                bl.setPower(slowmode*(turn + (vertical) - horizontal));
                br.setPower(slowmode*((-turn) + (vertical) + horizontal));
            }
            else {
                tl.setPower(0.7*(turn + (vertical) + horizontal));
                tr.setPower(0.7*((-turn) + (vertical) - horizontal));
                bl.setPower(0.7*(turn + (vertical) - horizontal));
                br.setPower(0.7*((-turn) + (vertical) + horizontal));
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
                lever.setPosition(0.9); //transfer position
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
            if (gamepad2.dpad_up) {
                arm.setTargetPosition(3300);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                sleep(3000);
            }
            if (gamepad2.dpad_down) {
                arm.setTargetPosition(420);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);
                sleep(3000);
            }

        }
    }
}

//new arm down