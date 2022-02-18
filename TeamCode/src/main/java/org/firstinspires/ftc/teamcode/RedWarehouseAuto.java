package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

    /**
     * This file illustrates the concept of driving a path based on encoder counts.
     * It uses the common Pushbot hardware class to define the drive on the robot.
     * The code is structured as a LinearOpMode
     *
     * The code REQUIRES that you DO have encoders on the wheels,
     *   otherwise you would use: PushbotAutoDriveByTime;
     *
     *  This code ALSO requires that the drive Motors have been configured such that a positive
     *  power command moves them forwards, and causes the encoders to count UP.
     *
     *   The desired path in this example is:
     *   - Drive forward for 48 inches
     *   - Spin right for 12 Inches
     *   - Drive Backwards for 24 inches
     *   - Stop and close the claw.
     *
     *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
     *  that performs the actual movement.
     *  This methods assumes that each movement is relative to the last stopping place.
     *  There are other ways to perform encoder based moves, but this method is probably the simplest.
     *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @Autonomous(name="Red Warehouse Auto", group="Pushbot")
    //@Disabled
    public class RedWarehouseAuto extends LinearOpMode {

        /* Declare OpMode members. */
        HardwareDrivetrain robot   = new HardwareDrivetrain();   // Use a Pushbot's hardware
        private ElapsedTime runtime = new ElapsedTime();

        static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);

        static final double     DRIVE_SPEED             = 0.60;
        static final double     TURN_SPEED              = 0.5;

        @Override
        public void runOpMode() {

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0",  "Starting at %7d :%7d",
                    robot.lfDrive.getCurrentPosition(),
                    robot.rfDrive.getCurrentPosition());
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            encoderDrive(DRIVE_SPEED,  17,  17, 6.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(TURN_SPEED,   19, -19, 8.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            encoderDrive(1.0, 60, 60, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeo

            sleep(1000);     // pause for servos to move

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

        /*
         *  Method to perform a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the opmode running.
         */
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


