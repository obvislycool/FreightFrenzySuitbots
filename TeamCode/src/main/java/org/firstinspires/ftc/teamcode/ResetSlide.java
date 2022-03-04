package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
@Autonomous(name = "ResetSlide", group = "Auto")
public class ResetSlide extends LinearOpMode {


    HardwareDrivetrain robot   = new HardwareDrivetrain();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Slide");    //
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();

        runtime.reset();
        robot.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.slideMotor.setPower(0.1);
        while(!robot.limitSensor.isPressed() && opModeIsActive() && runtime.seconds()<3){
            sleep(0);
        }
        robot.slideMotor.setPower(0);
        robot.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Slide Reset");    //
        telemetry.update();

    }
}