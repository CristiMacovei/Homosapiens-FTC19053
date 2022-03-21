package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="zeu", group = "Iterative Opmode")
public class Zeu extends OpMode {
    private boolean carouselTrigger = false;
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor carouselMotor = null;

    private Servo servo1 = null;
    private Servo servo2 = null;

    private DcMotor armMotor1 = null;

    private double MOTOR_THROTTLE = .8;
    private double ROTATION_THROTTLE = .8;
    private double position;

    @Override
    public void init() {
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

        carouselMotor.setDirection(DcMotor.Direction.REVERSE);


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        double xDist = - gamepad1.left_stick_x;
        double yDist = - gamepad1.left_stick_y;

        double leftPower = Range.clip(yDist + xDist, -1, 1) * MOTOR_THROTTLE;
        double rightPower = Range.clip(yDist - xDist, -1, 1) * MOTOR_THROTTLE;

        double rotateLeft = gamepad1.left_trigger*ROTATION_THROTTLE;
        double rotateRight = gamepad1.right_trigger*ROTATION_THROTTLE;

        if(leftPower!=0 && rightPower!=0){
            frontLeftMotor.setPower(rightPower);
            backLeftMotor.setPower(leftPower);

            frontRightMotor.setPower(leftPower);
            backRightMotor.setPower(rightPower);
        }

        if(leftPower == 0 && rightPower == 0 && rotateLeft == 0 && rotateRight == 0){
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);

            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }

        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);
        telemetry.addData("throttle", MOTOR_THROTTLE);
        telemetry.addData("rotation throttle", ROTATION_THROTTLE);
        telemetry.addData("position:", position);


        if (gamepad1.y) {
            telemetry.addData("PERIE", "REVERSE");
            servo1.setPosition(.3);
            servo2.setPosition(.3);
        }

        else if (gamepad1.b) {
            telemetry.addData("PERIE", "FORWARD");
            servo2.setPosition(1);
            servo1.setPosition(1);
        }

        else {
            telemetry.addData("PERIE", "STOP");
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
        }

        if (rotateLeft != 0) {
            telemetry.addData("rotate left", String.valueOf(rotateLeft));

            frontLeftMotor.setPower(-rotateLeft);
            frontRightMotor.setPower(rotateLeft);
            backLeftMotor.setPower(-rotateLeft);
            backRightMotor.setPower(rotateLeft);
        }



        if (rotateRight != 0) {
            telemetry.addData("rotate right", String.valueOf(rotateRight));

            frontLeftMotor.setPower(rotateRight);
            frontRightMotor.setPower(-rotateRight);
            backLeftMotor.setPower(rotateRight);
            backRightMotor.setPower(-rotateRight);
        }

        if (gamepad1.right_bumper) {
            MOTOR_THROTTLE = 1.0;
        } else if (gamepad1.left_bumper) {
            MOTOR_THROTTLE = 0.3;
        } else {
            MOTOR_THROTTLE = 0.7;
        }
        telemetry.addData("Speed Multiplier", MOTOR_THROTTLE);
        // if(gamepad1.left_bumper){
        //     position=armMotor1.getCurrentPosition();
        //     telemetry.addData("position", String.valueOf(position));
        //     armMotor1.setTargetPosition(450);
        //     armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     armMotor1.setPower(.3);
        // }
        // else if(gamepad1.right_bumper){
        //     position=armMotor1.getCurrentPosition();
        //     telemetry.addData("position", String.valueOf(position));
        //     armMotor1.setTargetPosition(0);
        //     armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     armMotor1.setPower(.3);
        //     // armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // }

        if (gamepad1.dpad_up) {
            armMotor1.setTargetPosition(0);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor1.setPower(.3);
            while(armMotor1.isBusy()){
                xDist = - gamepad1.left_stick_x;
                yDist = - gamepad1.left_stick_y;

                leftPower = Range.clip(yDist + xDist, -1, 1) * MOTOR_THROTTLE;
                rightPower = Range.clip(yDist - xDist, -1, 1) * MOTOR_THROTTLE;

                frontLeftMotor.setPower(rightPower);
                backLeftMotor.setPower(leftPower);

                frontRightMotor.setPower(leftPower);
                backRightMotor.setPower(rightPower);

                rotateLeft = gamepad1.left_trigger*ROTATION_THROTTLE;

                if (rotateLeft != 0) {
                    telemetry.addData("rotate left", String.valueOf(rotateLeft));

                    frontLeftMotor.setPower(-rotateLeft);
                    frontRightMotor.setPower(rotateLeft);
                    backLeftMotor.setPower(-rotateLeft);
                    backRightMotor.setPower(rotateLeft);
                }

                rotateRight = gamepad1.right_trigger*ROTATION_THROTTLE;

                if (rotateRight != 0) {
                    telemetry.addData("rotate right", String.valueOf(rotateRight));

                    frontLeftMotor.setPower(rotateRight);
                    frontRightMotor.setPower(-rotateRight);
                    backLeftMotor.setPower(rotateRight);
                    backRightMotor.setPower(-rotateRight);
                }
            }
            armMotor1.setPower(0);
            armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.dpad_right) {
            armMotor1.setTargetPosition(135);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor1.setPower(.3);
        }
        if (gamepad1.dpad_down) {
            armMotor1.setTargetPosition(275);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor1.setPower(.3);
        }
        if (gamepad1.dpad_left) {
            armMotor1.setTargetPosition(380);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor1.setPower(.3);
        }
        if (gamepad1.x) {
            carouselMotor.setPower(0.4);
        }
        else {
            carouselMotor.setPower(0);
        }
    }
}
