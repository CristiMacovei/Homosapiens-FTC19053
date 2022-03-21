package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="basic", group = "Iterative Opmode")
public class MotorTest extends OpMode {
    private boolean carouselTrigger = false;
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor carouselMotor = null;

    private Servo servo1 = null;
    private Servo servo2 = null;

    private DcMotor armMotor1 = null;

    private double MOTOR_THROTTLE = .7;
    private double ROTATION_THROTTLE = .7;
    private double position;

    @Override
    public void init() {
        telemetry.addData("MODE", "zeu");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "m_frontleft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "m_frontright");
        backLeftMotor = hardwareMap.get(DcMotor.class, "m_backleft");
        backRightMotor = hardwareMap.get(DcMotor.class, "m_backright");
        carouselMotor = hardwareMap.get(DcMotor.class, "m_carusel");

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        armMotor1 = hardwareMap.get(DcMotor.class, "m_arm1");
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setDirection(DcMotor.Direction.FORWARD);


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            frontLeftMotor.setPower(.7);
            backLeftMotor.setPower(.7);
            frontRightMotor.setPower(-.7);
            backRightMotor.setPower(-.7);
        }
        if(gamepad1.b){
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
}
