package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Dance")
public class Go_V2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot R = new Robot();
        R.init(hardwareMap);
        //waitForStart();
        R.reset();
        DcMotor lift_fd = null;
        DcMotor left_fd = null;
        DcMotor lift_up = null;
        DcMotor rt_fd = null;
        DcMotor left_bk = null;
        DcMotor rt_bk = null;
        left_fd = hardwareMap.get(DcMotor.class, "lt_fd");
        //left_fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rt_fd = hardwareMap.get(DcMotor.class, "rt_fd");
        //rt_fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_bk = hardwareMap.get(DcMotor.class, "lt_bk");
        //left_bk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rt_bk = hardwareMap.get(DcMotor.class, "rt_bk");
        lift_up = hardwareMap.get(DcMotor.class, "lift_up");
        lift_fd = hardwareMap.get(DcMotor.class, "lift_fd");
        lift_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double p_rt_bk = 0;
        float p_rt_fd = 0;
        float p_lt_fd = 0;
        float p_lt_bk = 0;
        Servo grab;
        Servo Slt;
        Servo Srt;
        grab = hardwareMap.get(Servo.class, "grab");
        Slt = hardwareMap.get(Servo.class, "Slt");
        Srt = hardwareMap.get(Servo.class, "Srt");
        waitForStart();

        //R.go_sec_with_imu(0, 0.3, 0.0, 4);
        Slt.setPosition(0.6);
         Srt.setPosition(0.4);
        Thread.sleep(450);
        Slt.setPosition(0.5);
        Srt.setPosition(0.5);
        lift_up.setPower(0.8);
        Thread.sleep(1000);
        lift_up.setPower(0);
        R.turn(45, 0.3);
            Thread.sleep(500);
            for(int r = 0; r  < 3; r++) {
                R.turn(-90, 0.3);
                Thread.sleep(500);
                R.turn(90, 0.3);
                Thread.sleep(500);
            }
            Thread.sleep(500);
            R.turn(-45, 0.3);
        lift_up.setPower(-0.6);
        Thread.sleep(1000);
        lift_up.setPower(0);
        for(int g = 0; g < 6; g++) {
            Thread.sleep(500);
            lift_up.setPower(0.8);
            Thread.sleep(300);
            lift_up.setPower(-0.15);
        }
        R.turn(45, 0.3);
            Thread.sleep(500);
            for(int r = 0; r  < 2; r++) {
                R.turn(-90, 0.3);
                Thread.sleep(500);
                R.turn(90, 0.3);
                Thread.sleep(500);
            }
            Thread.sleep(500);
            R.turn(-90, 0.3);
            Thread.sleep(500);
        Thread.sleep(500);
        R.turn(45, 0.3);
        Thread.sleep(500);
            R.reset();
        Thread.sleep(500);
        R.go_sec_IMU(0, 0, -0.4, 5.5);
        R.go_sec_IMU(0, 0.4, 0.1, 3.5);
        double mosh = 0.25 ;
        while(true){
            R.rt_bk.setPower(mosh);
            R.lt_fd.setPower(mosh);
            R.rt_fd.setPower(mosh);
            R.lt_bk.setPower(mosh);
            Thread.sleep(4000);
            R.rt_bk.setPower(-mosh);
            R.lt_fd.setPower(-mosh);
            R.rt_fd.setPower(-mosh);
            R.lt_bk.setPower(-mosh);
            Thread.sleep(4000);
        }
    }
}