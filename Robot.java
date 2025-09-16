package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BHI260IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    DcMotor lt_fd = null;
    DcMotor rt_fd = null;
    DcMotor lt_bk = null ;
    DcMotor rt_bk = null ;
    DcMotor lift_fd = null;
    DcMotor lift_up = null;
    Servo grab;
    Servo Slt;
    Servo Srt;
    BHI260IMU imu;
    ElapsedTime runtime = new ElapsedTime();
    double delta = 0;
    public double getAngle() {
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle*-1;
    }
    public void reset() {
        imu.resetYaw();
    }
    public void init(HardwareMap hardwareMap){
        lt_fd = hardwareMap.get(DcMotor.class, "lt_fd");
        lt_fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rt_fd = hardwareMap.get(DcMotor.class, "rt_fd");
        rt_fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lt_bk = hardwareMap.get(DcMotor.class, "lt_bk");
        lt_bk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rt_bk = hardwareMap.get(DcMotor.class, "rt_bk");
        rt_bk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_up = hardwareMap.get(DcMotor.class, "lift_up");
        lift_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_fd = hardwareMap.get(DcMotor.class, "lift_fd");
        lift_fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grab = hardwareMap.get(Servo.class, "grab");
        Slt = hardwareMap.get(Servo.class, "Slt");
        Srt = hardwareMap.get(Servo.class, "Srt");
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
        ));
        imu.resetYaw();
    }
    public void stop(){
        rt_bk.setPower(0);
        rt_fd.setPower(0);
        lt_bk.setPower(0);
        lt_fd.setPower(0);
    }
    public void go_sec_IMU(int ang, double y, double x, double sec){
        y=y*-1;
        double t = runtime.seconds();
        float p_rt_bk = 0;
        float p_rt_fd = 0;
        float p_lt_fd = 0;
        float p_lt_bk = 0;
        //getRuntime();
        while(((runtime.seconds() - t)*2) < sec){
            double er = ang - getAngle();
            float P = (float) (er*1.1 / 100);
            p_rt_bk = (float) (y - x);
            p_rt_fd = (float) (y + x);
            p_lt_fd = (float) (-y + x);
            p_lt_bk = (float) (-y - x);
            rt_bk.setPower(p_rt_bk + P);
            lt_fd.setPower(p_lt_fd + P);
            rt_fd.setPower(p_rt_fd + P);
            lt_bk.setPower(p_lt_bk + P);
        }
        stop();


    }
    void turn(double ang,double mosh, double kp){
        reset();
        if(ang > 0) {
            while (getAngle() < ang*kp) {
                rt_bk.setPower(mosh);
                lt_fd.setPower(mosh);
                rt_fd.setPower(mosh);
                lt_bk.setPower(mosh);
            }
        }else {
            while (getAngle() > ang) {
                rt_bk.setPower(-mosh);
                lt_fd.setPower(-mosh);
                rt_fd.setPower(-mosh);
                lt_bk.setPower(-mosh);
            }
        }
        stop();
    }
    public void stabel(double kp, double porog) {
        double angl_vozrat = delta*-1;
        delta = 0;
        reset();
        if (angl_vozrat > 0) {
            double er = angl_vozrat - getAngle();
            while (er > 1) {
                double P = (er * kp) / 100;
                rt_bk.setPower(P);
                lt_fd.setPower(P);
                rt_fd.setPower(P);
                lt_bk.setPower(P);
                er = angl_vozrat - getAngle();
                if (P < porog) {
                    er = 0;
                }
            }
        } else if (angl_vozrat < 0) {
            double er = angl_vozrat - getAngle();
            while (er < -1) {
                double P = (er * kp) / 100;
                rt_bk.setPower(P);
                lt_fd.setPower(P);
                rt_fd.setPower(P);
                lt_bk.setPower(P);
                er = angl_vozrat - getAngle();
                if (P > -porog) {
                    er = 0;
                }
            }
        }
        stop();
    }

    public void turn_P(double target_angl, double kp, double porog){
        //reset();
        delta = delta + getAngle();
        if (delta > 180) {
            delta = -180 + (delta - 180);
        }
        if (delta < -180) {
            delta = 180 + (delta + 180);
        }
        reset();
        if (target_angl > 0) {
            double er = target_angl - getAngle();
            while (er > 1) {
                double P = (er * kp) / 100;
                rt_bk.setPower(P);
                lt_fd.setPower(P);
                rt_fd.setPower(P);
                lt_bk.setPower(P);
                er = target_angl - getAngle();
                if (P < porog) {
                    er = 0;
                }
            }
        } else if (target_angl < 0) {
            double er = target_angl - getAngle();
            while (er < -1) {
                double P = (er * kp) / 100;
                rt_bk.setPower(P);
                lt_fd.setPower(P);
                rt_fd.setPower(P);
                lt_bk.setPower(P);
                er = target_angl - getAngle();
                if (P > -porog) {
                    er = 0;
                }
            }
        }
        stop();
        delta = delta + getAngle();
        if (delta > 180) {
            delta = -180 + (delta - 180);
        }
        if (delta < -180) {
            delta = 180 + (delta + 180);
        }
        stop();
    }
    public void go_grd_IMU(double y, double x, int target_grd, int target_ang){
        lt_bk.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rt_fd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lt_bk.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rt_fd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        float p_rt_bk = 0;
        float p_rt_fd = 0;
        float p_lt_fd = 0;
        float p_lt_bk = 0;
        y = -y;
        int lt_bk_enc = abs(lt_bk.getCurrentPosition());
        int rt_fd_enc = abs(lt_bk.getCurrentPosition());
        int enc_sr = (lt_bk_enc + rt_fd_enc) / 2;
        double er_grd = target_grd - enc_sr;
        double er = target_ang - getAngle();
        while (((er_grd * 100) / target_grd) > 5) { // в процентах !!!
            er = target_ang - getAngle();
            er_grd = target_grd - enc_sr;
            float P = (float) (er * 1.5 / 100);
            float P_grd = (float) (er_grd * 1.5 / 100);
            p_rt_bk = (float) (y - x);
            p_rt_fd = (float) (y + x);
            p_lt_fd = (float) (-y + x);
            p_lt_bk = (float) (-y - x);
            rt_bk.setPower(p_rt_bk + P );
            lt_fd.setPower(p_lt_fd + P );
            rt_fd.setPower(p_rt_fd + P );
            lt_bk.setPower(p_lt_bk + P );
            lt_bk_enc = abs(lt_bk.getCurrentPosition());
            rt_fd_enc = abs(lt_bk.getCurrentPosition());
            enc_sr = (lt_bk_enc + rt_fd_enc) / 2;
            /*telemetry.addData("Left_enc", lt_bk.getCurrentPosition());
            telemetry.addData("Right_enc", rt_fd.getCurrentPosition());
            telemetry.addData("sr_enc", enc_sr);
            telemetry.addData("er", er);
            telemetry.update();*/
        }
        stop();



    }
}
