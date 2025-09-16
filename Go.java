package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Teleopus")// объявление работы управляемого периода
public class Go extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot R = new Robot();
        R.init(hardwareMap);
        waitForStart();
        R.reset();
        Servo grab;
        grab = hardwareMap.get(Servo.class, "grab");
        float p_rt_bk = 0;
        float p_rt_fd = 0;
        float p_lt_fd = 0;
        float p_lt_bk = 0;
        while (opModeIsActive()) {
            //telemetry.addData("IMU", R.getAngle());
            //telemetry.addData("Delta", R.delta);
            //telemetry.update();
            if (gamepad1.dpad_right) {
                R.turn_P(90, 0.8, 0.09);
            }
            if (gamepad1.dpad_left) {
                R.turn_P(-90, 0.8, 0.09);
            }
            /*if (gamepad1.y) {
                R.stabel(0.8, 0.09); Stabel работоет криво, потом переписать
            }*/

            R.lift_up.setPower(gamepad2.left_stick_y*0.95);
            R.lift_fd.setPower(gamepad2.left_stick_x*0.6);
            //telemetry.addData("triger", gamepad2.left_trigger*45);
            //telemetry.addData("Delta", R.delta);
            //telemetry.update();
            R.Slt.setPosition(((gamepad2.right_stick_x - gamepad2.right_stick_y)*0.3)+0.5);
            R.Srt.setPosition(((gamepad2.right_stick_x + gamepad2.right_stick_y)*0.3)+0.5);
            grab.setPosition(gamepad2.left_trigger*0.5);
            if (gamepad1.right_bumper) {
                double er = 0 - R.getAngle();
                float P = (float) (er / 100);
                p_rt_bk = gamepad1.left_stick_y - gamepad1.left_stick_x;
                p_rt_fd = gamepad1.left_stick_y + gamepad1.left_stick_x;
                p_lt_fd = -gamepad1.left_stick_y + gamepad1.left_stick_x;
                p_lt_bk = -gamepad1.left_stick_y - gamepad1.left_stick_x;
                R.rt_bk.setPower(p_rt_bk * 0.8 + P);
                R.lt_fd.setPower(p_lt_fd * 0.8 + P);
                R.rt_fd.setPower(p_rt_fd * 0.8 + P);
                R.lt_bk.setPower(p_lt_bk * 0.8 + P);

            } else if (!gamepad1.right_bumper) {
                p_rt_bk = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                p_rt_fd = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                p_lt_fd = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                p_lt_bk = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                R.rt_bk.setPower(p_rt_bk * 0.8);
                R.lt_fd.setPower(p_lt_fd * 0.8);
                R.rt_fd.setPower(p_rt_fd * 0.8);
                R.lt_bk.setPower(p_lt_bk * 0.8);
            }
                //Thread.sleep(1);

        }
        R.stop();
        Thread.sleep(100);
    }
}
