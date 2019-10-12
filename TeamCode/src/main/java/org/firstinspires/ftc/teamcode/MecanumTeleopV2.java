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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MecanumTeleopV2", group="Linear Opmode")
//@Disabled
public class MecanumTeleopV2 extends LinearOpMode {

    private HardwarePushbot_BucketBrigade robot = new HardwarePushbot_BucketBrigade();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        robot.FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.BackRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //Forward/Backward
            robot.FrontLeftDrive.setPower(gamepad1.left_stick_y);
            robot.BackLeftDrive.setPower(gamepad1.left_stick_y);
            robot.FrontRightDrive.setPower(gamepad1.right_stick_y);
            robot.BackRightDrive.setPower(gamepad1.right_stick_y);

            //Left
            robot.FrontLeftDrive.setPower(gamepad1.left_trigger);
            robot.BackLeftDrive.setPower(-gamepad1.left_trigger);
            robot.FrontRightDrive.setPower(-gamepad1.left_trigger);
            robot.BackRightDrive.setPower(gamepad1.left_trigger);
            //Right
            robot.FrontLeftDrive.setPower(-gamepad1.right_trigger);
            robot.BackLeftDrive.setPower(gamepad1.right_trigger);
            robot.FrontRightDrive.setPower(gamepad1.right_trigger);
            robot.BackRightDrive.setPower(-gamepad1.right_trigger);

            // Send calculated power to wheels


            // Show the elapsed game time and wheel power.
           /* telemetry.addData("Status", "Run Time: " + runtime].toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", FleftPower, FrightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", BleftPower, BrightPower);
            telemetry.update();*/
           //Hello
        }
    }
}