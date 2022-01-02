package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="mecanumdrive", group="Linear Opmode")
public class mecanumdrive extends LinearOpMode {

    // Declare OpMode members.
    DcMotor tl;
    DcMotor tr;
    DcMotor bl;
    DcMotor br;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        tl = hardwareMap.get(DcMotor.class, "tl");
        tr = hardwareMap.get(DcMotor.class, "tr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        tl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        tr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);




        waitForStart();
        while (opModeIsActive()){
            double vertical;
            double horizontal;
            double turn;
            vertical = gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            tl.setPower(turn + (-vertical) + horizontal);
            tr.setPower((-turn) + (-vertical) - horizontal);
            bl.setPower(turn + (-vertical) - horizontal);
            br.setPower((-turn) + (-vertical) + horizontal);
        }
    }
}
