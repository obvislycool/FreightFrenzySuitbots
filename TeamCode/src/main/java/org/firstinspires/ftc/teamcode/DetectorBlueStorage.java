package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

//@Disabled //Comment out to run
@Autonomous(name = "DetectorBlueStorage", group = "Auto")
public class DetectorBlueStorage extends LinearOpMode {

    OpenCvWebcam webcam;
    private ElapsedTime runtime = new ElapsedTime();
    private int lCount;
    private int mCount;
    private int rCount;
    HardwareDrivetrain robot   = new HardwareDrivetrain();

    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     DRIVE_SPEED             = 0.60;
    static final double     TURN_SPEED              = 0.5;

    static final double DUCK_SPEED = 0.3;

    static final boolean CLOCKWISE = true;
    static final boolean COUNTERCLOCKWISE = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.lfDrive.getCurrentPosition(),
                robot.rfDrive.getCurrentPosition());
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ShippingElementDetector detector = new ShippingElementDetector(telemetry);
        webcam.setPipeline(detector);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() //Start streaming with the webcam
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });




        waitForStart();

        //while(!opModeIsActive)
        //if(opModeIsActive)
        int samples = 0;

        runtime.reset();
        while (opModeIsActive() && samples < 35 && runtime.seconds()<2.5){

            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addLine("In Loop");
            switch (detector.getElementPosition()){
                case LEFT:
                    lCount++;
                    telemetry.addLine("Position Detected: LEFT");
                    telemetry.update();
                    break;
                case MIDDLE:
                    mCount++;
                    telemetry.addLine("Position Detected: MIDDLE");
                    telemetry.update();
                    break;
                case RIGHT:
                    rCount++;
                    telemetry.addLine("Position Detected: RIGHT");
                    telemetry.update();
                    break;
                default:
                    rCount++;
                    telemetry.addLine("None");
                    telemetry.update();
                    break;
            }
            samples++;
            sleep(50);
        }
        telemetry.update();
        webcam.stopStreaming();

        encoderDrive(TURN_SPEED, -8, 8, 2); //turn to hub
        encoderDrive(DRIVE_SPEED,  22,  22, 3);  //drive to hub
        encoderDrive(TURN_SPEED,   -44, +44, 4);  //180 turn
        encoderDrive(0.35, -9.75, -9.75, 3);  //back into hub
        //duckSpin(COUNTERCLOCKWISE,6000);
        //encoderDrive(DRIVE_SPEED,  -19,  19, 6.0);
        //encoderDrive(DRIVE_SPEED,  9,  9, 6.0);

        if(lCount >= mCount && lCount >= rCount){ //Set slide to correct position and dump
            telemetry.addLine("RUNNING LEFT AUTO");
            telemetry.update();
            slide(500);
            dump(0);
            robot.dumpServo.setPosition(45); //reset dumper
            slide(-500);
        }else if (rCount >= mCount){
            telemetry.addLine("RUNNING RIGHT AUTO");
            telemetry.update();
            slide(2300);
            dump(0);
            robot.dumpServo.setPosition(45); //reset dumper
            slide(-2300);
        }else{
            telemetry.addLine("RUNNING MIDDLE AUTO");
            telemetry.update();
            slide(1400);
            dump(0);
            robot.dumpServo.setPosition(45); //reset dumper
            slide(-1400);
        }

        encoderDrive(DRIVE_SPEED, 2, 2, 2);
        encoderDrive(TURN_SPEED, -4, 4, 1);
        encoderDrive(DRIVE_SPEED, 42, 42, 5);  // go to wheel
        duckSpin(COUNTERCLOCKWISE,6000);
        encoderDrive(TURN_SPEED,   -27, 27, 3);
        encoderDrive(DRIVE_SPEED,  23,  23, 3);
        //encoderDrive(DRIVE_SPEED,  -19,  19, 6.0);
        //encoderDrive(DRIVE_SPEED,  9,  9, 6.0);

        //JANKY STRAFE


        robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rfDrive.setTargetPosition(-400);
        robot.lfDrive.setTargetPosition(-400);
        robot.rbDrive.setTargetPosition(400);
        robot.lbDrive.setTargetPosition(400);

        robot.rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rfDrive.setPower(0.4);
        robot.lfDrive.setPower(0.4);
        robot.rbDrive.setPower(0.4);
        robot.lbDrive.setPower(0.4);

        while (opModeIsActive() && robot.rfDrive.isBusy() && robot.lfDrive.isBusy() && robot.rbDrive.isBusy() && robot.lbDrive.isBusy()) {
            sleep(0);
        }
        robot.rfDrive.setPower(0);
        robot.lfDrive.setPower(0);
        robot.rbDrive.setPower(0);
        robot.lbDrive.setPower(0);

        robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        sleep(500);
        telemetry.addData("Path", "Complete");
        telemetry.update();


    }
    public void dump(double position){
        robot.dumpServo.setPosition(position);
        sleep(3000);
    }

    public void slide(int ticks){
        robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideMotor.setTargetPosition(ticks);
        robot.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideMotor.setPower(0.3);
        while (opModeIsActive()&& robot.slideMotor.isBusy()) {
            sleep(0);
        }
        robot.slideMotor.setPower(0);
        robot.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void duckSpin(boolean direction, double milliseconds){
        //double timeAtStart = getRuntime();
        robot.duckDrive.setPower((direction) ? DUCK_SPEED : -DUCK_SPEED);
        sleep((long) milliseconds);
        /*while(getRuntime() - timeAtStart < milliseconds){ //lasts seconds seconds
        }*/
        robot.duckDrive.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLFTarget;
        int newRFTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = robot.lfDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRFTarget = robot.rfDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.lfDrive.setTargetPosition(newLFTarget);
            robot.rfDrive.setTargetPosition(newRFTarget);
            robot.lbDrive.setTargetPosition(newLFTarget);
            robot.rbDrive.setTargetPosition(newRFTarget);


            // Turn On RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lfDrive.setPower(Math.abs(speed));
            robot.rfDrive.setPower(Math.abs(speed));
            robot.lbDrive.setPower(Math.abs(speed));
            robot.rbDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.lfDrive.isBusy() && robot.rfDrive.isBusy() && robot.lbDrive.isBusy() && robot.rbDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLFTarget,  newRFTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.lfDrive.getCurrentPosition(),
                        robot.rfDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.lfDrive.setPower(0);
            robot.rfDrive.setPower(0);
            robot.lbDrive.setPower(0);
            robot.rbDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
