package org.firstinspires.ftc.teamcode;

//Importing required classes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Declaring autonomous named Servo_Autonomous with the ground test
@Disabled
@Autonomous(name="Blue Foundation", group="Blue")
// Creating class named servo autonomous that uses linear op mode
public class Blue_Foundation extends LinearOpMode {

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
    HolonomicDrive holonomicDrive;

    // Starting OPMode
    @Override
    public void runOpMode() {

        // Classifying our Dc motors with their names on the expansion hub
        FrontRightMotor  = hardwareMap.get(DcMotor.class, "front_right_drive");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        BackRightMotor  = hardwareMap.get(DcMotor.class, "back_right_drive");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");

        // Setting our holonomic drive to use our 2 front and 2 back motors
        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);

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
        holonomicDrive.autoDrive(0, 0.95);
        // Running a while loop that is active for our set amount of time which is 1.4 seconds in this case
        while (opModeIsActive() && runtime.seconds() < 1.5){
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
        while (opModeIsActive() && runtime.seconds() < 0.8) {
            // Adding telemetry data with the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stop robot removed if the program doesn't work correctly
        // Resetting the time again
        runtime.reset();
        // Driving backwards at a max speed of 50% for 1.9 seconds
        holonomicDrive.autoDrive(180, 0.95);
        // Running a while loop so the robot will not try to do anything until it has moved
        // backwards for 1.9 seconds
        while (opModeIsActive() && runtime.seconds() < 2.0){
            // Adding telemetry of the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stopping the robot so it doesn't continue to drive into the wall
        holonomicDrive.stopMoving();
        // Resetting the time again
        runtime.reset();
        // Moving the hooks so they will not collide with the foundation
        left_hook.setPosition(lStored);
        right_hook.setPosition(rStored);

        holonomicDrive.autoDrive(90,0.95);
        while (opModeIsActive() && runtime.seconds() < 3.){
            // Adding telemetry of the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stopping the robot so it doesn't continue to drive into the wall
        holonomicDrive.stopMoving();


    }


}
