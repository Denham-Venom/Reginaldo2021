// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{

//Motors defined here
	public static final int MOTOR_TOP_LEFT_ID = 1;
	public static final int MOTOR_BOTTOM_LEFT_ID = 2;
	public static final int MOTOR_TOP_RIGHT_ID = 3;
	public static final int MOTOR_BOTTON_RIGHT_ID = 4;
	
	public static final int SHOOT_MOTOR_LEFT_ID = 5;
	public static final int SHOOT_MOTOR_RIGHT_ID = 6;

	public static final int INDEXER_MOTOR_1 = 7; //Left
	public static final int INDEXER_MOTOR_2 = 8; //Right

	public static final int INTAKE_MOTOR_ID = 10; // spark #1
	public static final int SPIN_UP_MOTOR = 12; // spark #2
	public static final int ADJUST_ANGLE_MOTOR_LEFT = 11; // spark #3
	public static final int ADJUST_ANGLE_MOTOR_RIGHT = 9; // spark #4	

	
//-----------------------------------------------------    
//Buttons/Axis defined here
	public static final int XBUTTON = 1;
	public static final int YBUTTON = 4;
	public static final int ABUTTON = 2;
	public static final int BBUTTON = 3;
	public static final int RBBUTTON = 6;
	public static final int LBBUTTON = 5;
	public static final int rtBUTTON = 8;
	public static final int ltBUTTON = 7;

	public static final int RX_AXIS = 2;
	public static final int LY_AXIS = 1;
//-----------------------------------------------------

//Controller defined here
	public static final int PILOT_DRIVER_CONTROLLER = 0;
//-----------------------------------------------

//High Gear & Low Gear
	public static final double DT_HG = 1;
	public static final double DT_HGT = 0.9;
	public static final double DT_LG = 0.8;
	public static final double DT_LGT = 0.65;
//-----------------------------------------------

//Speed Constants
	public static final double ANGLE_MOTOR_SPEED = 0.1;
	public static final double SHOOTER_MOTOR_SPEED = 0.45; //maybe set lower
	public static final double INDEXER_SPEED = 0.9;
	public static final double SPIN_UP_SPEED = 0.8;
	public static final double INTAKE_BALL_SPEED = 0.9;
	public static final double MAX_MP_VELOCITY = 0.5;
	public static final double MAX_MP_ACCELERATION = 0.5;
//-----------------------------------------------
//Garbage PID, FF, & ERRRRRRROR here 
	public static final double DT_PID_F = 0;
	public static final double DT_PID_P = 0.015;
	public static final double DT_PID_I = 0;
	public static final double DT_PID_D = 0;
	public static final double DT_PID_FT_P = 0.1;
	public static final double DT_TURN_F = 0.15;
	public static final double DT_TURN_P = 0.005;
	public static final double DT_TURN_I = 0;
	public static final double DT_TURN_D = 0;
	
	public static final double ERR = 2.0;
	public static final double DT_MOVE_ERR = .1;
	public static final double ALLOWABLE_STEER_ERR = 1;
	public static final double ALLOWABLE_AIM_ERR = .25;

	public static final double SHOOTER_AIM_ANG_FF = 3.5;
	public static final double FEEDFORWARD = 0.009;

	public static final int kTimeoutMS = 30;
	public static final int kPIDLoopIdx = 0;
	
	public static final int kTimeMS = 0;
	public static final int kSlotIdx = 0;
	public static final double allowableCloseLoopError = 0.25; //drivetrain movement in feet
//------------------------------------------------------
//Conversions yes b/c icedso
	public static final double ADJUST_ROTS_TO_DEGR = 3. * 1260./25. / 42.;
	public static final double ADJUST_FF = 0.01;
	public static final double FEET_TO_ROT_UNITS = (2048) * (50 / 24) * (2 / Math.PI) * (5);//(Ticks per rev) * (gear ratio) / (rev/ft)  c=6pi in = pi/2 ft
	public static final double RPM_TO_TP100MS = 2048./600. * 18./14.; // OutRevs/Min * Min/Sec * Sec/100ms * Ticks/InRev * InRev/OutRev
	public static final double ENCODER_TICKS_TO_FEET = 1 / FEET_TO_ROT_UNITS;
//---------------------------------------------------------------------------
	


	public static final double LL_ANG = 10;
	public static final double CAM_ANG_TO_SHOOT_ANG = 1.18;
	public static final double SHOOT_ADJ_VOLT_LIM = .2; 
	public static final double SHOOTER_MOTOR_VELOCITY = 2500; //RPM...allegedly



	public static final double PEAK_SHOOTER = 0.6;
	public static final int kAUXLoopIdx = 1;
	public static final double SHOOT_AIM_P = 0;
	public static final double SHOOT_AIM_I = 0;
	public static final double SHOOT_AIM_D = 0;
	public static final double SHOOT_AIM_F = 0.09;
	
	










}
