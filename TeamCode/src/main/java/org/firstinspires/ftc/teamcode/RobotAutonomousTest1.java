package org.firstinspires.ftc.teamcode;

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

@Autonomous(name="RobotAutonomousTest1", group="Pushbot")
public class RobotAutonomousTest1 extends LinearOpMode {
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

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            "AdHwNjn/////AAABmdhNlo7iF0zIjJ96OYWNdeMgLYqjp7kNnO8YW2SwcYwHZjXaAWLBtLHGgiEQUohwzbPQO55eiXFPIXqwa9fI5RKXbxqcL5FJvGoLs4EEZaebutnORhRkqhRk/KKBsG6nC0VNrsXu9DaqQmpEc3FENATOg/K2I4Z3IhA69AqJHXhkIQovnl6kcT9l6y6MSOwRJ6PZrPbuII0GOZgTJgvaAmXwvQqt0moa7fz8yABRPTd0hhZL7ax2lrRJpVtYv4SM8KHCVZMUG4bQfdR4C8cFFdWCRZ6tmCxqIPo02oLvO6l2MroieHtLUrAlBmuULotpaweWyu5am/mRYeTG9h1+KwlHWsyrzpzx86+vX3dORPzq";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

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

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.69f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
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

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.2, 21.0/9.0);
        }

         telemetry.addData(">", "Press Play to start op mode");


        waitForStart();
        int level=3;

        // 1 = [108, 303.5]
        // 2 = [312.5, 287]
        // 3 = [512, 271.5]

        double pos1 = 108;
        double pos2 = 312.5;
        double pos3 = 512;

        while (opModeIsActive()) {
            telemetry.update();
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    boolean found = false;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equalsIgnoreCase("duck") || recognition.getLabel().equalsIgnoreCase("duck")) {
                            double cx = (recognition.getLeft() + recognition.getRight()) / 2;
                            telemetry.addData("found @", String.valueOf(cx));
                            telemetry.update();
                            double dist1 = Math.abs(cx - pos1);
                            double dist2 = Math.abs(cx - pos2);
                            double dist3 = Math.abs(cx - pos3);

                            if (dist1 < dist2 && dist1 < dist3)
                                level = 1;
                            else if (dist2 < dist1 && dist2 < dist3)
                                level = 2;
                            else
                                level = 3;

                            telemetry.addData("poz", String.valueOf(level));
                            telemetry.update();
                            found = true;
                            break;
                        }

                        telemetry.addLine(String.valueOf(found));

                        // telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        // telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        //         recognition.getLeft(), recognition.getTop());
                        // telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        //         recognition.getRight(), recognition.getBottom());
                        // i++;

                    }
                    if(found){
                        break;
                    }
                }
            }
        }


        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        moveYAxis(300, mPower);
        moveXAxis(1120, mPower);
        moveYAxis(560, mPower); // initial: 940
        armAuto(level);
        moveYAxis(-480, mPower);
        moveXAxis(-1900, mPower);
        moveXAxis(-220, 0.2);
        carouselMotor.setPower(0.3);
        sleep(5000);
        carouselMotor.setPower(0);
        moveXAxis(600, 0.5);

        rotate(750, 0.3);
        moveXAxis(600, 0.5);
        moveYAxis(3500, 0.8);
        moveXAxis(-600, .8);
        //moveYAxis(-3500, .8);
        // telemetry.addData("position ", testMotor.getTargetPosition());
        // telemetry.addData("time ", runtime.seconds());
        // telemetry.update();
        // sleep(10000);
    }
}