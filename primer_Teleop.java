//облочка для запуска программы
package org.firstinspires.ftc.teamcode;

//импортируем пакет для работы с электроникой
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//Импортируем класс TeleOp(Управляемого периуда)
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//Импортируем класс моторов для управления моторами
import com.qualcomm.robotcore.hardware.DcMotor;
//Импортируем класс серво для работы с ним
import com.qualcomm.robotcore.hardware.Servo;

//Устанавливаем тип программы как TeleOp(Управляемого периуда) и задаем его название
@TeleOp(name = "Primer_TeleOp")
//Объявляем класс наследуюший LinerOpMode
public class primer_Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // создаем переменные для каждого мотора, где будет считаться мощность мотора
        double p_rt_fd = 0;
        double p_lt_fd = 0;
        double p_lt_bk = 0;
        double p_rt_bk = 0;

        //Config-файл на драйвер хабе где записанны порты куда подключенны моторы,сервы и датчики
        //Создаем объект мотора с названием lt_fd(левый передний) и читаем объект из configa
        DcMotor lt_fd = hardwareMap.get(DcMotor.class, "lt_fd");
        //Устанавливаем режим левого переднего мотора как удержания позиции при остановке
        lt_fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Создаем объект мотора с названием rt_fd(правый передний) и читаем объект из configa
        DcMotor rt_fd = hardwareMap.get(DcMotor.class, "rt_fd");
        //Устанавливаем режим правого переднего мотора как удержания позиции при остановке
        rt_fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Создаем объект мотора с названием lt_bk(левый задний) и читаем объект из configa
        DcMotor lt_bk = hardwareMap.get(DcMotor.class, "lt_bk");
        //Устанавливаем режим левого заднего мотора как удержания позиции при остановке
        lt_bk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Создаем объект мотора с названием rt_fd(правый задний) и читаем объект из configa
        DcMotor rt_bk = hardwareMap.get(DcMotor.class, "rt_bk");
        //Устанавливаем режим правого заднего мотора как удержания позиции при остановке
        rt_bk.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Создаем объект мотора с названием lift_up(мотор подъма лифта) и читаем объект из configa
        DcMotor lift_up = hardwareMap.get(DcMotor.class, "lift_up");
        //Устанавливаем режим мотора подъёма лифта как удержания позиции при остановке
        lift_up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Создаем объект мотора с названием lift_up(мотор выдвигания лифта) и читаем объект из configa
        DcMotor lift_fd = hardwareMap.get(DcMotor.class, "lift_fd");
        //Устанавливаем режим мотора выдвигания лифта как удержания позиции при остановке
        lift_fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Создаем объект серва с названием grab(серво клешни) и читаем объект из configa
        Servo grab = hardwareMap.get(Servo.class, "grab");

        //Ждем нажатия кнопки старт на Драйвер хабе
        waitForStart();
        // условия пока программа работает
        while (opModeIsActive()) {
            //задаем позицию серво с тригера(l2)
            // серво на вход принимает тип данных double от 0 до 1, где 0 - 0, 1 - 180
            // Тригер выдает значения типа double в диапозоне от 0 до 1
            grab.setPosition(gamepad2.left_trigger * 0.5);
            // устанвливаем мощность мотора подъма лифта с 2 gamepada левой оси y
            // все гаймпады выдают значения типа double от 0 до 1
            lift_up.setPower(gamepad2.left_stick_y * 0.95);
            // устанвливаем мощность мотора выдвигания лифта с 2 gamepada левой оси x
            lift_fd.setPower(gamepad2.left_stick_x * 0.6);
            //каждый джостик выдает значения типа double от -1 до 1
            //для каждого мотора вычисляем мощность в зависимости от значения джостиков
            p_rt_bk = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            p_rt_fd = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            p_lt_fd = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            p_rt_bk = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            //задаем мощность на моторы и домнажаем на 0.8 для уменьшения скорости
            rt_bk.setPower(p_rt_bk * 0.8);
            lt_fd.setPower(p_lt_fd * 0.8);
            rt_fd.setPower(p_rt_fd * 0.8);
            lt_bk.setPower(p_lt_bk * 0.8);

        }
        // После отключения программы останавливаем моторы
        rt_bk.setPower(0);
        rt_fd.setPower(0);
        lt_bk.setPower(0);
        lt_fd.setPower(0);
        // И ждем для коректной работы
        Thread.sleep(100);
    }
}