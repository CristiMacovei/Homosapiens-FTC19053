package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="hexmotortest")
public class HexMotorTest extends OpMode {

    DcMotor hex;

    @Override
    public void init() {
        hex = hardwareMap.get(DcMotor.class, "motorBazat");
    }

    @Override
    public void loop() {
        hex.setPower(.69);
    }
}
