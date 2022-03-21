package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;


@Autonomous(name="robotauto2", group="Pushbot")
public class RobotAuto2 extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor carouselMotor = null;

    private Servo servo1 = null;
    private Servo servo2 = null;

    private DcMotor armMotor1 = null;

    private double mPower = 0.5;

    private static final String VUFORIA_KEY =
            "AdHwNjn/////AAABmdhNlo7iF0zIjJ96OYWNdeMgLYqjp7kNnO8YW2SwcYwHZjXaAWLBtLHGgiEQUohwzbPQO55eiXFPIXqwa9fI5RKXbxqcL5FJvGoLs4EEZaebutnORhRkqhRk/KKBsG6nC0VNrsXu9DaqQmpEc3FENATOg/K2I4Z3IhA69AqJHXhkIQovnl6kcT9l6y6MSOwRJ6PZrPbuII0GOZgTJgvaAmXwvQqt0moa7fz8yABRPTd0hhZL7ax2lrRJpVtYv4SM8KHCVZMUG4bQfdR4C8cFFdWCRZ6tmCxqIPo02oLvO6l2MroieHtLUrAlBmuULotpaweWyu5am/mRYeTG9h1+KwlHWsyrzpzx86+vX3dORPzq";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

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

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void moveYAxis (int position, double power) {
        frontLeftMotor.setTargetPosition(position);
        frontRightMotor.setTargetPosition(position);
        backLeftMotor.setTargetPosition(position);
        backRightMotor.setTargetPosition(position);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        while (frontLeftMotor.isBusy()) {

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveXAxis (int position, double power) {
        frontLeftMotor.setTargetPosition(position);
        frontRightMotor.setTargetPosition(-position);
        backLeftMotor.setTargetPosition(-position);
        backRightMotor.setTargetPosition(position);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        while (frontLeftMotor.isBusy()) {

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void rotate (int position, double power) {
        frontLeftMotor.setTargetPosition(position);
        frontRightMotor.setTargetPosition(-position);
        backLeftMotor.setTargetPosition(position);
        backRightMotor.setTargetPosition(-position);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);

        while (frontLeftMotor.isBusy()) {

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void armAuto (int level) {
        if (level == 1) {
            armMotor1.setTargetPosition(135);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor1.setPower(.3);
        }
        if (level == 2) {
            armMotor1.setTargetPosition(275);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor1.setPower(.3);
        }
        if (level == 3) {
            armMotor1.setTargetPosition(380);
            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor1.setPower(.3);
        }

        while(armMotor1.isBusy()) {

        }

        if (level == 3) {
            moveYAxis(175, mPower);
        }


        servo2.setPosition(.7);
        servo1.setPosition(.7);

        sleep(2500);

        servo2.setPosition(0.5);
        servo1.setPosition(0.5);
        if (level == 3) {
            moveYAxis(-175, mPower);
        }
        moveYAxis(-200, 0.3);
        armMotor1.setTargetPosition(0);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1.setPower(.3);
        while(armMotor1.isBusy());
        armMotor1.setPower(0);
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "m_frontleft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "m_frontright");
        backLeftMotor = hardwareMap.get(DcMotor.class, "m_backleft");
        backRightMotor = hardwareMap.get(DcMotor.class, "m_backright");
        carouselMotor = hardwareMap.get(DcMotor.class, "m_carusel");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        armMotor1 = hardwareMap.get(DcMotor.class, "m_arm1");


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        carouselMotor.setDirection(DcMotor.Direction.REVERSE);


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initVuforia();
        telemetry.addLine("inited camera");
        telemetry.update();

        /** Wait for the game to begin */
        // telemetry.addData(">", "Press Play to start op mode");


        waitForStart();
        int level=3;

        telemetry.addData(">", "Amogus sex");
        telemetry.update();

        // 1 = [108, 303.5]
        // 2 = [312.5, 287]
        // 3 = [512, 271.5]

        double pos1 = 108;
        double pos2 = 312.5;
        double pos3 = 512;

        vuforia.setFrameQueueCapacity(1000);

        while (opModeIsActive()) {
            try {
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
                Image rgb = null;

                long numImages = frame.getNumImages();
                telemetry.addData("numImages", frame );
                telemetry.update();
                long[] numPixelsFound = new long[3];
                for (int i = 0; i < numImages; ++i) {
//                    telemetry.addData("awdaw", frame.getImage(i).getFormat());
//                    telemetry.addData("egrsgtgtsgstgts", PIXEL_FORMAT.RGB565 );
                    telemetry.update();
                    if (frame.getImage(i).getFormat() == 4) {
//                        telemetry.addData("am intrat","in if" );
                        telemetry.update();
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

                        telemetry.addData("left", numPixelsFound[0]);
                        telemetry.addData("mid", numPixelsFound[1]);
                        telemetry.addData("right", numPixelsFound[2]);
                        telemetry.update();
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
                    telemetry.addData("level", level);
                    telemetry.update();
                    break;
                }
            } catch (Exception ex) {
                telemetry.addData("Caught error", ex);
                telemetry.update();
            }
        }

        telemetry.addData("level", level);
        telemetry.update();

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // moveYAxis(300, mPower);
        // moveXAxis(1120, mPower);
        // moveYAxis(560, mPower); // initial: 940
        // armAuto(level);
        // moveYAxis(-480, mPower);
        // moveXAxis(-1900, mPower);
        // moveXAxis(-220, 0.2);
        // carouselMotor.setPower(0.3);
        // sleep(5000);
        // carouselMotor.setPower(0);
        // moveXAxis(600, 0.5);

        // rotate(750, 0.3);
        // moveXAxis(600, 0.5);
        // moveYAxis(3500, 0.8);
        // moveXAxis(-600, .8);
    }
}