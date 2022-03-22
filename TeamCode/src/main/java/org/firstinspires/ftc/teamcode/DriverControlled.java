package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="HomosapiensTeleOp", group = "Iterative Opmode")
public class DriverControlled extends OpMode {
    private double MOTOR_THROTTLE = .8;
    private double ROTATION_THROTTLE = .8;
    private double position;

    @Override
    public void init() {
        RobotConfig.init(hardwareMap);


        RobotConfig.Motors.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotConfig.Motors.arm.setDirection(DcMotor.Direction.FORWARD);

        RobotConfig.Motors.carousel.setDirection(DcMotor.Direction.REVERSE);


        RobotConfig.Motors.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        RobotConfig.Motors.backLeft.setDirection(DcMotor.Direction.REVERSE);

        RobotConfig.Motors.frontRight.setDirection(DcMotor.Direction.FORWARD);
        RobotConfig.Motors.backRight.setDirection(DcMotor.Direction.FORWARD);

        RobotConfig.Motors.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotConfig.Motors.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotConfig.Motors.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotConfig.Motors.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RobotConfig.Servos.brush1.setDirection(Servo.Direction.FORWARD);
        RobotConfig.Servos.brush2.setDirection(Servo.Direction.REVERSE);

        RobotConfig.Motors.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //todo move the code in this function
    void handleMovement() {
        double xDist = - gamepad1.left_stick_x;
        double yDist = - gamepad1.left_stick_y;

        double leftPower = Range.clip(yDist + xDist, -1, 1) * MOTOR_THROTTLE;
        double rightPower = Range.clip(yDist - xDist, -1, 1) * MOTOR_THROTTLE;

        double rotateLeft = gamepad1.left_trigger*ROTATION_THROTTLE;
        double rotateRight = gamepad1.right_trigger*ROTATION_THROTTLE;

        //
        if (leftPower == 0 && rightPower == 0 && rotateLeft == 0 && rotateRight == 0) {
            telemetry.addData("Set all power to", "0");

            RobotConfig.Motors.setAllPower(0);
        }

        // left rotation
        if (rotateLeft != 0) {
            telemetry.addData("rotate left", String.valueOf(rotateLeft));

            RobotConfig.Motors.rotateLeft(rotateLeft);

            return;
        }

        // right rotation
        if (rotateRight != 0) {
            telemetry.addData("rotate right", String.valueOf(rotateRight));

            RobotConfig.Motors.rotateRight(rotateRight);

            return;
        }

        // movement
        if (leftPower != 0 && rightPower != 0) {
            telemetry.addData("Set left power to", String.valueOf(leftPower));
            telemetry.addData("Set right power to", String.valueOf(rightPower));

            RobotConfig.Motors.setLeftPower(leftPower);
            RobotConfig.Motors.setRightPower(rightPower);
        }

    }

    @Override
    public void loop() {
        handleMovement();

        if (gamepad1.y) {
            telemetry.addData("PERIE", "REVERSE");
            RobotConfig.Servos.brush1.setPosition(.3);
            RobotConfig.Servos.brush2.setPosition(.3);
        }

        else if (gamepad1.b) {
            telemetry.addData("PERIE", "FORWARD");
            RobotConfig.Servos.brush2.setPosition(1);
            RobotConfig.Servos.brush1.setPosition(1);
        }

        else {
            telemetry.addData("PERIE", "STOP");
            RobotConfig.Servos.brush1.setPosition(0.5);
            RobotConfig.Servos.brush2.setPosition(0.5);
        }

        if (gamepad1.right_bumper) {
            MOTOR_THROTTLE = 1.0;
        } else if (gamepad1.left_bumper) {
            MOTOR_THROTTLE = 0.3;
        } else {
            MOTOR_THROTTLE = 0.7;
        }

        telemetry.addData("Speed Multiplier", MOTOR_THROTTLE);

        if (gamepad1.dpad_up) {
            RobotConfig.Motors.arm.setTargetPosition(0);
            RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotConfig.Motors.arm.setPower(.3);

            while(RobotConfig.Motors.arm.isBusy()){
                handleMovement();
            }

            RobotConfig.Motors.arm.setPower(0);
            RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.dpad_right) {
            RobotConfig.Motors.arm.setTargetPosition(135);
            RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotConfig.Motors.arm.setPower(.3);
        }
        if (gamepad1.dpad_down) {
            RobotConfig.Motors.arm.setTargetPosition(275);
            RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotConfig.Motors.arm.setPower(.3);
        }
        if (gamepad1.dpad_left) {
            RobotConfig.Motors.arm.setTargetPosition(380);
            RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotConfig.Motors.arm.setPower(.3);
        }

        if (gamepad1.x) {
            RobotConfig.Motors.carousel.setPower(0.4);
        }
        else {
            RobotConfig.Motors.carousel.setPower(0);
        }
    }
}
