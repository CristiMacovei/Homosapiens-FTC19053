/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name="autonomiecustomelement")
public class AutoRedNear extends LinearOpMode {
    private double mPower = 0.4;
    private static final String VUFORIA_KEY =
            "AdHwNjn/////AAABmdhNlo7iF0zIjJ96OYWNdeMgLYqjp7kNnO8YW2SwcYwHZjXaAWLBtLHGgiEQUohwzbPQO55eiXFPIXqwa9fI5RKXbxqcL5FJvGoLs4EEZaebutnORhRkqhRk/KKBsG6nC0VNrsXu9DaqQmpEc3FENATOg/K2I4Z3IhA69AqJHXhkIQovnl6kcT9l6y6MSOwRJ6PZrPbuII0GOZgTJgvaAmXwvQqt0moa7fz8yABRPTd0hhZL7ax2lrRJpVtYv4SM8KHCVZMUG4bQfdR4C8cFFdWCRZ6tmCxqIPo02oLvO6l2MroieHtLUrAlBmuULotpaweWyu5am/mRYeTG9h1+KwlHWsyrzpzx86+vX3dORPzq";

    private VuforiaLocalizer vuforia;

    public void moveYAxis (int position, double power) {
        RobotConfig.Motors.frontLeft.setTargetPosition(position);
        RobotConfig.Motors.frontRight.setTargetPosition(position);
        RobotConfig.Motors.backLeft.setTargetPosition(position);
        RobotConfig.Motors.backRight.setTargetPosition(position);

        RobotConfig.Motors.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RobotConfig.Motors.frontLeft.setPower(power);
        RobotConfig.Motors.frontRight.setPower(power);
        RobotConfig.Motors.backLeft.setPower(power);
        RobotConfig.Motors.backRight.setPower(power);

        while (RobotConfig.Motors.frontLeft.isBusy()) {

        }

        RobotConfig.Motors.frontLeft.setPower(0);
        RobotConfig.Motors.frontRight.setPower(0);
        RobotConfig.Motors.backLeft.setPower(0);
        RobotConfig.Motors.backRight.setPower(0);

        RobotConfig.Motors.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveXAxis (int position, double power) {
        RobotConfig.Motors.frontLeft.setTargetPosition(position);
        RobotConfig.Motors.frontRight.setTargetPosition(-position);
        RobotConfig.Motors.backLeft.setTargetPosition(-position);
        RobotConfig.Motors.backRight.setTargetPosition(position);

        RobotConfig.Motors.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RobotConfig.Motors.frontLeft.setPower(power);
        RobotConfig.Motors.frontRight.setPower(power);
        RobotConfig.Motors.backLeft.setPower(power);
        RobotConfig.Motors.backRight.setPower(power);

        while (RobotConfig.Motors.frontLeft.isBusy()) {

        }

        RobotConfig.Motors.frontLeft.setPower(0);
        RobotConfig.Motors.frontRight.setPower(0);
        RobotConfig.Motors.backLeft.setPower(0);
        RobotConfig.Motors.backRight.setPower(0);

        RobotConfig.Motors.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void rotate (int position, double power) {
        RobotConfig.Motors.frontLeft.setTargetPosition(position);
        RobotConfig.Motors.frontRight.setTargetPosition(-position);
        RobotConfig.Motors.backLeft.setTargetPosition(position);
        RobotConfig.Motors.backRight.setTargetPosition(-position);

        RobotConfig.Motors.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RobotConfig.Motors.frontLeft.setPower(power);
        RobotConfig.Motors.frontRight.setPower(power);
        RobotConfig.Motors.backLeft.setPower(power);
        RobotConfig.Motors.backRight.setPower(power);

        while (RobotConfig.Motors.frontLeft.isBusy()) {

        }

        RobotConfig.Motors.frontLeft.setPower(0);
        RobotConfig.Motors.frontRight.setPower(0);
        RobotConfig.Motors.backLeft.setPower(0);
        RobotConfig.Motors.backRight.setPower(0);

        RobotConfig.Motors.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void armAuto (int level) {
        if (level == 1) {
            RobotConfig.Motors.arm.setTargetPosition(135);
            RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotConfig.Motors.arm.setPower(.3);
        }
        if (level == 2) {
            RobotConfig.Motors.arm.setTargetPosition(275);
            RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotConfig.Motors.arm.setPower(.3);
        }
        if (level == 3) {
            RobotConfig.Motors.arm.setTargetPosition(380);
            RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RobotConfig.Motors.arm.setPower(.3);
        }

        while(RobotConfig.Motors.arm.isBusy()) {

        }

        if (level == 3) {
            moveYAxis(175, mPower);
        }


        RobotConfig.Servos.brush2.setPosition(.7);
        RobotConfig.Servos.brush1.setPosition(.7);

        sleep(2500);

        RobotConfig.Servos.brush2.setPosition(0.5);
        RobotConfig.Servos.brush1.setPosition(0.5);
        if (level == 3) {
            moveYAxis(-175, mPower);
        }
        moveYAxis(-200, 0.3);
        RobotConfig.Motors.arm.setTargetPosition(0);
        RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotConfig.Motors.arm.setPower(.3);
        while(RobotConfig.Motors.arm.isBusy());
        RobotConfig.Motors.arm.setPower(0);
        RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void runOpMode() {
        RobotConfig.init(hardwareMap);

        RobotConfig.Motors.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotConfig.Motors.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotConfig.Motors.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotConfig.Motors.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RobotConfig.Motors.carousel.setDirection(DcMotor.Direction.REVERSE);


        RobotConfig.Motors.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        RobotConfig.Motors.backLeft.setDirection(DcMotor.Direction.REVERSE);

        RobotConfig.Motors.frontRight.setDirection(DcMotor.Direction.FORWARD);
        RobotConfig.Motors.backRight.setDirection(DcMotor.Direction.FORWARD);

        RobotConfig.Servos.brush1.setDirection(Servo.Direction.REVERSE);
        RobotConfig.Servos.brush2.setDirection(Servo.Direction.FORWARD);

        RobotConfig.Motors.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotConfig.Motors.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotConfig.Motors.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotConfig.Motors.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RobotConfig.Motors.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RobotConfig.Motors.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initVuforia();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press start when ready.");
        telemetry.update();
        waitForStart();

        int level=3;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                try {
                    VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
                    Image rgb = null;

                    long numImages = frame.getNumImages();

                    long[] numPixelsFound = new long[3];

                    for (int i = 0; i < numImages; ++i) {
                        if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                            Image img = frame.getImage(i);

                            Bitmap map = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                            map.copyPixelsFromBuffer(img.getPixels());

                            final double maxDelta = 50;
                            for (int side = 0; side < 3; ++side) {
                                int start = side * (img.getWidth() / 3);
                                int end = (1 + side) * (img.getWidth() / 3);

                                for (int x = start; x < end; ++x) {
                                    for (int y = 0; y < img.getHeight(); ++y) {
                                        int pixel = map.getPixel(x, y);

                                        final int red = Color.red(pixel);
                                        final int green = Color.green(pixel);
                                        final int blue = Color.blue(pixel);

                                        if (
                                                maxDelta >= Math.abs(255 -   red) &&
                                                        maxDelta >= Math.abs(128 - green) &&
                                                        maxDelta >= Math.abs(  0 -  blue)
                                        ) {
                                            ++numPixelsFound[side];
                                        }
                                    }
                                }
                            }
                            telemetry.addData("Orange pixels in segments:", "");
                            telemetry.addData("Left Segment", numPixelsFound[0]);
                            telemetry.addData("Mid Segment", numPixelsFound[1]);
                            telemetry.addData("Right Segment", numPixelsFound[2]);
                        }
                    }

                    long max = 1000;
                    int index = -1;
                    for (int side = 0; side < 3; ++side) {
                        if (numPixelsFound[side] >= max) {
                            max = numPixelsFound[side];
                            index = side;
                        }
                    }

                    if (index != -1) {
                        level = index + 1;
                        telemetry.addData("Level detected", level);
                        telemetry.update();
                        break;
                    }
                    telemetry.addData("Ran succesfully", "No errors encountered");
                } catch (Exception ex) {
                    telemetry.addData("Error thrown", ex);
                }

                telemetry.update();
            }
        }

        // telemetry.addData("level", level);
        // telemetry.update();

        RobotConfig.Motors.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotConfig.Motors.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sleep(30000);

        moveYAxis(300, mPower);
        moveXAxis(1200, mPower);
        moveYAxis(530, mPower); // initial: 940
        armAuto(level);
        moveYAxis(-440, mPower);
        moveXAxis(-1900, mPower);
        moveXAxis(-265, 0.2);
        RobotConfig.Motors.carousel.setPower(0.3);
        sleep(5000);
        RobotConfig.Motors.carousel.setPower(0);
        moveXAxis(600, 0.5);

        rotate(750, 0.4);
        moveXAxis(600, 0.5);
        moveYAxis(3500, 0.9);
        moveXAxis(-600, 0.8);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }
}
