package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotConfig {
    public static class Motors {
        public static DcMotor frontLeft = null;
        public static DcMotor frontRight = null;
        public static DcMotor backLeft = null;
        public static DcMotor backRight = null;

        public static DcMotor carousel = null;

        public static DcMotor arm = null;

        public static void setLeftPower(double power) {
            backLeft.setPower(power);
            frontRight.setPower(power);
        }

        public static void setRightPower(double power) {
            frontLeft.setPower(power);
            backRight.setPower(power);
        }

        public static void setAllPower(double power) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }

        public static void rotateLeft(double power) {
            RobotConfig.Motors.frontLeft.setPower(-power);
            RobotConfig.Motors.frontRight.setPower(power);
            RobotConfig.Motors.backLeft.setPower(-power);
            RobotConfig.Motors.backRight.setPower(power);
        }

        public static void rotateRight(double power) {
            rotateLeft(-power);
        }
    }

    public static class Servos {
        public static Servo brush1 = null;
        public static Servo brush2 = null;
    }

    public static void init (HardwareMap hardwareMap) {
        Motors.frontLeft = hardwareMap.get(DcMotor.class, "m_frontleft");
        Motors.frontRight = hardwareMap.get(DcMotor.class, "m_frontright");
        Motors.backLeft = hardwareMap.get(DcMotor.class, "m_backleft");
        Motors.backRight = hardwareMap.get(DcMotor.class, "m_backright");

        Motors.arm = hardwareMap.get(DcMotor.class, "m_arm1");
        Motors.carousel = hardwareMap.get(DcMotor.class, "m_carusel");

        Servos.brush1 = hardwareMap.get(Servo.class, "servo1");
        Servos.brush2 = hardwareMap.get(Servo.class, "servo2");
    }
}
