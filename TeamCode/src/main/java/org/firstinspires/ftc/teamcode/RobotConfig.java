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
    }

    public static class Servos {
        public static Servo sweeper1 = null;
        public static Servo sweeper2 = null;
    }

    public static void init (HardwareMap hardwareMap) {
        Motors.frontLeft = hardwareMap.get(DcMotor.class, "m_frontleft");
        Motors.frontRight = hardwareMap.get(DcMotor.class, "m_frontright");
        Motors.backLeft = hardwareMap.get(DcMotor.class, "m_backleft");
        Motors.backRight = hardwareMap.get(DcMotor.class, "m_backright");

        Motors.arm = hardwareMap.get(DcMotor.class, "m_arm1");
        Motors.carousel = hardwareMap.get(DcMotor.class, "m_carusel");

        Servos.sweeper1 = hardwareMap.get(Servo.class, "servo1");
        Servos.sweeper2 = hardwareMap.get(Servo.class, "servo2");
    }
}
