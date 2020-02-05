package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;






@Autonomous(name = "Blue Bridge", group = "Blue")
public class Blue_Bridge extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor;
    private Servo left_hook, right_hook;
    HolonomicDrive holonomicDrive;
    BotServos bot_servo;
    double lStored = 0;
    double rStored = 1;


    @Override
    public void runOpMode() {

        FrontRightMotor = hardwareMap.get(DcMotor.class, "front_right_drive");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        BackRightMotor = hardwareMap.get(DcMotor.class, "back_right_drive");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");

        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);
        bot_servo = new BotServos(left_hook, right_hook);


        // Classifying our servos with their names on the expansion hub
        left_hook = hardwareMap.servo.get("left_hook");
        right_hook = hardwareMap.servo.get("right_hook");

        // Setting servos to the retracted position allowing them to move over the foundation lip
            left_hook .setPosition(lStored);
            right_hook.setPosition(rStored);

        waitForStart();

        runtime.reset();
        holonomicDrive.autoDrive(90,0.8);
        while (opModeIsActive() && runtime.seconds() < 1.0){
            // Adding telemetry of the time elapsed
            telemetry.addData("Path", "TIME: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        holonomicDrive.stopMoving();
    }
}

