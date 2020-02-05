package org.firstinspires.ftc.teamcode;

//Importing required classes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.gyro;
import com.qualcomm.hardware.bosch.BNO055IMU;


/**
 *
 *
 * May add a variable or another class that will park the robot on the left or right half of the alliance bridge
 *
 *
 * */
// Declaring autonomous named Servo_Autonomous with the ground test
@Autonomous(name="Red Foundation Gyro Outside", group="Red")
// Creating class named servo autonomous that uses linear op mode
public class Red_Foundation_Gyro extends LinearOpMode {

    double lStored = 0;
    double rStored = 1;
    double lActive = 0.6;
    double rActive = 0.4;


    // Declaring servos attached/required for this autonomous class
    Servo left_hook, right_hook;
    //Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();

    // Declaring the DC motors for our holonomic drive base
    private DcMotor FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor;
    BNO055IMU               imu;
    Orientation lastAngles = new Orientation();
    HolonomicDrive holonomicDrive;
    gyro Gyro;

    // Starting OPMode
    @Override
    public void runOpMode() {

        // Classifying our Dc motors with their names on the expansion hub
        FrontRightMotor  = hardwareMap.get(DcMotor.class, "front_right_drive");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        BackRightMotor  = hardwareMap.get(DcMotor.class, "back_right_drive");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Setting our holonomic drive to use our 2 front and 2 back motors
        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);
        Gyro = new gyro(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor, imu);

        // Classifying our servos with their names on the expansion hub
        left_hook = hardwareMap.servo.get("left_hook");
        right_hook = hardwareMap.servo.get("right_hook");

        // Setting servos to the retracted position allowing them to move over the foundation lip
        left_hook .setPosition(lStored);
        right_hook.setPosition(rStored);

        // Waiting for the player to hit the play button on the drive station phone when autonomous starts
        waitForStart();



        // Resetting our runtime variable which we will use to measure how long a process has been running
        runtime.reset();
        // Using the autodrive function from holonomic drive to drive the robot forwards at a max speed of 50%
        holonomicDrive.autoDrive(90, 0.75);
        // Running a while loop that is active for our set amount of time which is 1.4 seconds in this case
        while (opModeIsActive() && runtime.seconds() < 0.75){
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stopping the robot so the servos can activate before we return to the starting position
        holonomicDrive.stopMoving();
        runtime.reset();
        // Using the autodrive function from holonomic drive to drive the robot forwards at a max speed of 50%
        holonomicDrive.autoDrive(0, 0.75);
        // Running a while loop that is active for our set amount of time which is 1.4 seconds in this case
        while (opModeIsActive() && runtime.seconds() < 2.0){
            // Adding telemetry data of our direction and run time
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stopping the robot so the servos can activate before we return to the starting position
        holonomicDrive.stopMoving();
        // Resetting the time again
        runtime.reset();
        // Moving the servo hooks to grab onto the foundation
        left_hook.setPosition(lActive);
        right_hook.setPosition(rActive);
        // Running a while loop that will wait for 1 second before moving
        while (opModeIsActive() && runtime.seconds() < 0.7) {
            // Adding telemetry data with the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Resetting the time again
        runtime.reset();
        // Driving backwards
        holonomicDrive.autoDrive(180, 0.90);
        // Running a while loop so the robot will not try to do anything until it has moved
        // backwards for 1.0 second
        while (opModeIsActive() && runtime.seconds() < 1.5){
            // Adding telemetry of the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stopping the robot so it doesn't continue to drive
        holonomicDrive.stopMoving();
        Gyro.rotate(-90,0.5);
        sleep(500);
        runtime.reset();
        // We wil now drive towards the wall to push the foundation into the building site
        holonomicDrive.autoDrive(0, 0.95);
        // Running a while
        // loop so the robot will not try to do anything until it has moved
        while (opModeIsActive() && runtime.seconds() < 1.0){
            // Adding telemetry of the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stopping the robot so it doesn't continue to drive
        holonomicDrive.stopMoving();
        runtime.reset();
        // Moving the hooks so they will not collide with the foundation
        left_hook.setPosition(lStored);
        right_hook.setPosition(rStored);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.0){
            // Adding telemetry of the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // We will now want to stafe into our alliance wall to stay out of our alliance partners way
        runtime.reset();
        holonomicDrive.autoDrive(90, 0.95);
        while (opModeIsActive() && runtime.seconds() < 0.8){
            // Adding telemetry of the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stopping the robot so it doesn't continue to drive into the wall
        holonomicDrive.stopMoving();
        // Finally we want to drive and park under the alliance bridge
        runtime.reset();
        holonomicDrive.autoDrive(180, 0.95);
        // Running a while loop so the robot will not try to do anything until it has finished moving
        while (opModeIsActive() && runtime.seconds() < 2.2){
            // Adding telemetry of the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stopping the robot so it doesn't continue to drive
        holonomicDrive.stopMoving();


    }


}