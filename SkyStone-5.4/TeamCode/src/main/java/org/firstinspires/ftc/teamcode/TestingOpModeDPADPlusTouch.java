/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Testing Touch OpMode", group="Iterative Opmode")
//@Disabled
public class TestingOpModeDPADPlusTouch extends OpMode
{
    // Declare OpMode members.
    // Declare All Variables of Motors and Servos Used
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor  FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor, IntakeLeftMotor, IntakeRightMotor, ScissorLiftMotorLeft, ScissorLiftMotorRight;
    private Servo IntakePulley, left_hook, right_hook , claw , wrist;
    private CRServo expansion;

    //Declare the Subsystems used in this OpMode
    HolonomicDrive holonomicDrive;
    ScissorLift scissorLift;
    Intake_Systems intake_systems;
    BotServos bot_servo;
    ArmCollection armCollection;
// Declare the Touch Sensor
    DigitalChannel touch;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Initialize all the Hardware Devices and find their location on the Expansion Hub
        FrontRightMotor  = hardwareMap.get(DcMotor.class, "front_right_drive");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_drive");
        BackRightMotor  = hardwareMap.get(DcMotor.class, "back_right_drive");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "back_left_drive");
        IntakeLeftMotor = hardwareMap.get(DcMotor.class, "left_intake");
        IntakeRightMotor = hardwareMap.get(DcMotor.class, "right_intake");
        ScissorLiftMotorLeft =  hardwareMap.get(DcMotor.class, "scissor_left");
        ScissorLiftMotorRight =  hardwareMap.get(DcMotor.class, "scissor_right");

        IntakePulley = hardwareMap.servo.get("intake_pulley");
        left_hook = hardwareMap.servo.get("left_hook");
        right_hook = hardwareMap.servo.get("right_hook");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");
        expansion = hardwareMap.get(CRServo.class,"expansion");

        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT); // set the digital channel to input.

//Construct the subsystems using the declared & initialized hardware devices
        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);
        scissorLift = new ScissorLift(ScissorLiftMotorLeft, ScissorLiftMotorRight, IntakePulley);
        intake_systems = new Intake_Systems(IntakeRightMotor, IntakeLeftMotor, IntakePulley, touch, claw);
        bot_servo = new BotServos(left_hook, right_hook);
        armCollection = new ArmCollection(claw, wrist, expansion, IntakePulley);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Set Up All GameControllers

        //Drive
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;
        double slow = gamepad1.right_trigger;

        //Collector
        boolean collect1 = gamepad1.a;
        boolean deploy1 = gamepad1.b;
        boolean release_collection2 = gamepad2.a;
        boolean pullback_collection2 = gamepad2.b;

        //Foundation Servos
        boolean foundation_control1 = gamepad1.right_bumper; //foundation movers
        boolean foundation_control2 = gamepad1.left_bumper;

        //Scissor Lift
        double lift = gamepad2.left_stick_y;

        //Expansion Arm
        boolean dpad_up = gamepad2.dpad_up;
        boolean dpad_down = gamepad2.dpad_down;

        //Claw
        boolean grab_control = gamepad2.x;
        boolean stone_release_control = gamepad2.y;
        double wrist_control = gamepad2.right_stick_x;
        boolean fineWristRight = gamepad2.right_bumper;
        boolean fineWristLeft = gamepad2.left_bumper;
        boolean fineWristMiddle = gamepad2.start;

        //Running the subsystems with the controls

        //Drive
        holonomicDrive.teleopDrive(x,y,z, slow);
        //Collection
        intake_systems.inTel(collect1, deploy1);
        intake_systems.pullBackCollectionArms(pullback_collection2);
        intake_systems.releaseCollectionArms(release_collection2);
        // Foundation Servos
        bot_servo.retract(foundation_control2);
        bot_servo.activate(foundation_control1);
        //Scissor Lift
        scissorLift.LiftControl(-lift);
       //Expansion Arm
        armCollection.expandControlDPAD(dpad_up, dpad_down);
        //Claw
        armCollection.grab(grab_control);
        armCollection.release(stone_release_control);
        armCollection.wristControl(wrist_control, fineWristRight, fineWristLeft, fineWristMiddle);

        //Testing if the Touch Sensor was pressed which will signal a comment saying that the stone has been collected.
        //This touch sensor is used to see if the stone has been collected when we the robot is on the opposite side of
        // the field and we can't see whether or not we collected the stone or capstone.
        if (intake_systems.getTouch())
            telemetry.addData("The Stone's been collected", "");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
