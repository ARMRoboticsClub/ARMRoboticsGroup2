/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class Group2Auto extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
	// TETRIX VALUES.
	final static double ARM_MIN_RANGE  = 0.40;
	final static double ARM_MAX_RANGE  = 1.0;
	final static double CLAW_MIN_RANGE  = 0.30;
	final static double CLAW_MAX_RANGE  = 0.9;
	final static double MOTOR_POWER = 0.15; // Higher values will cause the robot to move faster
	private enum State
	{
		STATE_INITIAL,
		STATE_WAIT_FOR_TOUCH,
		STATE_DRIVE_TO_WALL,
		STATE_STOP,
	}
	// Loop cycle time stats variables
	public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.
	private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
	private State       mCurrentState;    // Current State Machine State.

	// position of the arm servo.
	double armPosition;

	// amount to change the arm servo position.
	double armDelta = 0.1;

	// position of the claw servo
	double clawPosition;

	// amount to change the claw servo position by
	double clawDelta = 0.1;

	DcMotor motorRight;
	DcMotor motorLeft;
	Servo claw;
	Servo arm;
	ServoController sc;
	TouchSensor sensor_touch;
	/**
	 * Constructor
	 */
	public Group2Auto() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot and reversed.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
		 */
		motorRight = hardwareMap.dcMotor.get("motor_2");
		motorLeft = hardwareMap.dcMotor.get("motor_1");
		motorRight.setDirection(DcMotor.Direction.REVERSE);

		// enable pwm.
		///sc
		//sc = hardwareMap.servoController.get("matrixServo");
		//sc.pwmEnable();

	//	arm = hardwareMap.servo.get("servo_1");
	//	claw = hardwareMap.servo.get("servo_6");
		sensor_touch = hardwareMap.touchSensor.get ("sensor_touch");
		// assign the starting position of the wrist and claw
		armPosition = 0.2;
		clawPosition = 0.2;
	}
	@Override
	public void start()
	{
		// Setup Robot devices, set initial state and start game clock
		mRuntime.reset();           // Zero game clock
		newState(State.STATE_INITIAL);
	}
	//--------------------------------------------------------------------------
	// loop
	//--------------------------------------------------------------------------
	@Override
	public void loop()
	{
		// Send the current state info (state and time) back to first line of driver station telemetry.
		telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());

		// Execute the current state.  Each STATE's case code does the following:
		// 1: Look for an EVENT that will cause a STATE change
		// 2: If an EVENT is found, take any required ACTION, and then set the next STATE
		//   else
		// 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
		//
		switch (mCurrentState)
		{
			case STATE_INITIAL:         // Stay in this state until encoders are both Zero.
				//do something
				newState(State.STATE_WAIT_FOR_TOUCH);  // Next State:

				break;

			case STATE_WAIT_FOR_TOUCH: // Follow path until last segment is completed
				if (sensor_touch.isPressed())
				{

					newState(State.STATE_DRIVE_TO_WALL);      // Next State:
				}
				else
				{
					// Display Diagnostic data for this state.
					telemetry.addData("1", String.format("WAIT_FOR_TOUCH"));
				}
				break;

			case STATE_DRIVE_TO_WALL: // Follow path until last segment is completed
				if (foundWall())
				{

					newState(State.STATE_STOP);      // Next State:
				}
				else
				{
					// Display Diagnostic data for this state.
					telemetry.addData("1", String.format("DRIVE_TO_WALL"));
				}
				break;
			case STATE_STOP:
				break;
		}
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
//	@Override
//	public void loop() {
//		double left;
//		double right;
//
//		if (sensor_touch.isPressed()) {
//
//
//
//			     /*
//				 *
//				 * Move forward
//				 */
//			left = MOTOR_POWER;
//			right = MOTOR_POWER;
//
//		} else {
//			/*
//			 * Shut off motors
//			 */
//			left = 0.0;
//			right = 0.0;
//		}
//
//		/*
//		 * set the motor power
//		 */
//		motorRight.setPower(right);
//		motorLeft.setPower(left);
//
//		/*
//		 * Send telemetry data back to driver station. Note that if we are using
//		 * a legacy NXT-compatible motor controller, then the getPower() method
//		 * will return a null value. The legacy NXT-compatible motor controllers
//		 * are currently write only.
//		 */
//        telemetry.addData("Text", "*** Robot Data***");
//        telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
//        telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
//        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
//        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
//
//	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}


	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}
	private boolean foundWall()
	{
		return true;
	}
	private void newState(State newState)
	{
		// Reset the state time, and then change to next state.
		mStateTime.reset();
		mCurrentState = newState;
	}

}
