package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name = "TestDetectorAuto", group = "Auto")
public class TestDetectorAuto extends LinearOpMode {

    OpenCvWebcam webcam;
    private ElapsedTime runtime = new ElapsedTime();
    private int lCount;
    private int mCount;
    private int rCount;

    @Override
    public void runOpMode() throws InterruptedException {
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


        while (!isStarted()){//FIXME
            telemetry.addData("Percent left","");
            telemetry.addData("Percent middle","");
            telemetry.addData("Percent right","");
            telemetry.addData("Position", "");
            telemetry.update();

        }

        waitForStart();

        //while(!opModeIsActive)
        //if(opModeIsActive)
        int samples = 0;

        runtime.reset();
        while (opModeIsActive() && samples < 50 && runtime.seconds()<8){

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
                    lCount++;
                    telemetry.addLine("None");
                    telemetry.update();
                    break;
            }
            samples++;
        }
        telemetry.update();

        if(lCount >= mCount && lCount >= rCount){ //Set elementPosition to correct position
            telemetry.addLine("RUNNING LEFT AUTO");
            telemetry.update();
        }else if (rCount >= mCount){
            telemetry.addLine("RUNNING RIGHT AUTO");
            telemetry.update();
        }else{
            telemetry.addLine("RUNNING MIDDLE AUTO");
            telemetry.update();
        }

        sleep(300000000);

        webcam.stopStreaming();
    }
}
