package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Test")// объявление работы управляемого периода
public class test_go extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot R = new Robot();
        R.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_right){
            R.turn(90, 0.7);
        } else if (gamepad1.dpad_left) {
                R.turn(-90, 0.7);
            }

            }
            //R.go_grd_IMU(0.5, 0,1000, 0);

    }
}
