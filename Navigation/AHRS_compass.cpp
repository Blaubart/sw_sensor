/** ***********************************************************************
 * @file		AHRS.cpp
 * @brief		AHRS Implementation: maintain aircraft attitude
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#include <AHRS.h>
#include <AHRS_compass.h>
#include "system_configuration.h"
#include "my_assert.h"
#include "GNSS.h"
#include "embedded_memory.h"

#define P_GAIN 0.03f			//!< Attitude controller: proportional gain
#define I_GAIN 0.00006f 		//!< Attitude controller: integral gain
#define H_GAIN 38.0f			//!< Attitude controller: horizontal gain
#define M_H_GAIN -10.0f			//!< Attitude controller: horizontal gain magnetic
#define CROSS_GAIN 0.05f		//!< Attitude controller: cross-product gain

#define HIGH_TURN_RATE 0.15 		//!< turn rate high limit
#define LOW_TURN_RATE  0.0707 		//!< turn rate low limit

    // inklin   = 66; deklin   = 3;
    // N=cos( inklin * pi/180)
    // E=sin( deklin * pi/180) * sin( inklin * pi/180)
    // D=sin( inklin * pi/180)];

extern compass_calibration_t *p_compass_calibration;

/**
 * @brief initial attitude setup from observables
*/
void AHRS_compass_type::attitude_setup( const float3vector & acceleration, const float3vector & induction)
{
	float3vector north, east, down;

	down  = acceleration;
	down.negate();
	north = induction;

	down.normalize();
	north.normalize();

	// setup world coordinate system
	east  = down.vector_multiply(north);
	east.normalize();
	north = east.vector_multiply(down);
	north.normalize();

	// create rotation matrix from unity direction vectors
	float fcoordinates[]=
	{
			north.e[0], north.e[1], north.e[2],
			 east.e[0],  east.e[1],  east.e[2],
			 down.e[0],  down.e[1],  down.e[2]
	};

	float3matrix coordinates( fcoordinates);
	attitude.from_rotation_matrix(coordinates);
	attitude.get_rotation_matrix( body2nav);
	body2nav.transpose( nav2body);
	euler = attitude;
}

/**
 * @brief  decide about circling state
*/
circle_state_t AHRS_compass_type::update_circling_state( const float3vector &gyro)
{
	float turn_rate_abs=abs( turn_rate);

	if( (circling_counter < CIRCLE_LIMIT) && ( turn_rate_abs > HIGH_TURN_RATE))
	    ++circling_counter;

	if(( circling_counter > 0) && ( turn_rate_abs < LOW_TURN_RATE))
	    --circling_counter;

	if( circling_counter == 0)
	  circle_state = STRAIGHT_FLIGHT;
	else if ( circling_counter == CIRCLE_LIMIT)
	  circle_state = CIRCLING;
	else
	  circle_state = TRANSITION;

	return circle_state;
}

/**
 * @brief  generic update of AHRS
 *
 * Side-effect: create rotation matrices, NAV-acceleration, NAV-induction
 */
void AHRS_compass_type::update( const float3vector &acc, const float3vector &gyro, const float3vector &mag)
{
	attitude.rotate(
			gyro.e[ROLL] * Ts_div_2,
			gyro.e[NICK] * Ts_div_2 ,
			gyro.e[YAW]  * Ts_div_2 );

	attitude.normalize();

	attitude.get_rotation_matrix( body2nav);
	body2nav.transpose( nav2body);

	acceleration_nav_frame 	= body2nav * acc;
	induction_nav_frame 	= body2nav * mag;
	euler=attitude;

	float3vector nav_rotation;
	nav_rotation = body2nav * gyro;
	turn_rate = nav_rotation[DOWN];

	slip_angle_averager.respond( my_atan2f( -acc.e[RIGHT], -acc.e[DOWN]));
	nick_angle_averager.respond( my_atan2f(  acc.e[FRONT],  acc.e[DOWN]));
}

/**
 * @brief  update attitude from IMU data and magnetometer
 */
void AHRS_compass_type::update_compass(
		const float3vector &gyro,
		const float3vector &acc,
		const float3vector &_mag,
	   	const float3vector &GNSS_acceleration
		)
{
	  float3vector mag;
	  if( p_compass_calibration) // use calibration if available
	    {
	      mag = p_compass_calibration->calibrate(_mag);
	    }
	  else
	    mag = _mag;

	  float3vector nav_acceleration = body2nav * acc;
	  float3vector nav_induction    = body2nav * mag;

	  // MAG vector projected on a normalized horizontal plane and normalized
	  nav_induction[DOWN] = 0;
	  nav_induction.normalize();

	  // calculate horizontal leveling error
	  nav_correction[NORTH] = - nav_acceleration.e[EAST]  + GNSS_acceleration.e[EAST];
	  nav_correction[EAST]  = + nav_acceleration.e[NORTH] - GNSS_acceleration.e[NORTH];

	  // *******************************************************************************************************
	  // calculate heading error depending on the present circling state
	  // on state changes handle MAG auto calibration
	  circle_state_t new_state = update_circling_state (gyro);
	  switch( new_state)
	  {
	    case STRAIGHT_FLIGHT:
	    {
	      nav_correction[DOWN] = nav_induction[EAST] * M_H_GAIN; // todo hier fehlt magnet-modell erde
	      gyro_correction = nav2body * nav_correction;
	      gyro_correction *= P_GAIN;

	      // update integrator and use it
	      gyro_integrator += gyro_correction;
	      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
	    }
	    break;
	    // *******************************************************************************************************
	    case CIRCLING:
	    {
	      float cross_correction = // vector cross product GNSS-acc und INS-acc -> heading error
        	    nav_acceleration.e[NORTH] * GNSS_acceleration.e[EAST]
		  - nav_acceleration.e[EAST]  * GNSS_acceleration.e[NORTH];

              nav_correction[DOWN]  =   cross_correction * CROSS_GAIN; // no MAG use here !
              nav_correction[DOWN] = 0.5f * (nav_correction[DOWN] + nav_induction[EAST] * M_H_GAIN);

	      gyro_correction = nav2body * nav_correction;

	      // don't update integrator but use it
	      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
	      gyro_correction *= P_GAIN;
	    }
	    break;
	    // *******************************************************************************************************
	    case TRANSITION:
	    {
	      nav_correction[DOWN] = nav_induction[EAST] * M_H_GAIN;

	      gyro_correction = nav2body * nav_correction;
	      gyro_correction *= P_GAIN;

	      // don't update integrator but use it
	      gyro_correction = gyro_correction + gyro_integrator * I_GAIN;
	    }
	    break;
	    default:
	      ASSERT(0);
	    break;
	  }

	  // feed quaternion update with corrected sensor readings
	  update (acc, gyro + gyro_correction, mag);
}

