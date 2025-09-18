package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test")// объявление работы управляемого периода
public class test_go extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot R = new Robot();
        R.init(hardwareMap);
        R.reset();
        waitForStart();
        double p_rt_bk = 0;
        float p_rt_fd = 0;
        float p_lt_fd = 0;
        float p_lt_bk = 0;
        while(opModeIsActive()) {
            if (gamepad1.dpad_right) {
                R.turn(90, 0.5, 0.9);
            } else if (gamepad1.dpad_left) {
                R.turn(-90, 0.5, 0.9);
            }
            if (gamepad1.dpad_down){
                R.go_grd_IMU(0.6, 0,500, 0);
            }
            p_rt_bk = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            p_rt_fd = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            p_lt_fd = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            p_lt_bk = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            R.rt_bk.setPower(p_rt_bk * 0.8);
            R.lt_fd.setPower(p_lt_fd * 0.8);
            R.rt_fd.setPower(p_rt_fd * 0.8);
            R.lt_bk.setPower(p_lt_bk * 0.8);

            //R.go_grd_IMU(0.5, 0,1000, 0);
        }
    }
}
