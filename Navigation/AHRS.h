/** ***********************************************************************
 * @file		AHRS.h
 * @brief		AHRS header
 * @author		Dr. Klaus Schaefer
 **************************************************************************/

#ifndef AHRS_H_
#define AHRS_H_

typedef float ftype;
//typedef float ftype;

#include "quaternion.h"
#include "float3vector.h"
#include "float3matrix.h"
#include "integrator.h"

#define SQR(x) ((x)*(x))
#define SIN(x) sinf(x)
#include "pt2.h"

enum { ROLL, NICK, YAW};
enum { FRONT, RIGHT, BOTTOM};
typedef enum  { STRAIGHT_FLIGHT, TRANSITION, CIRCLING} circle_state_t;

#define ANGLE_F_BY_FS ( 1.0 / 0.5f / 100.0f)      // 0.5s

typedef integrator<float, float3vector> vector3integrator;

//! Attitude and heading reference system class
class AHRS_type
{
public:
	AHRS_type(float sampling_time)
		:
		  Ts(sampling_time),
		  Ts_div_2 (sampling_time / 2.0),
		  gyro_integrator({0}),
		  circling_counter(0),
		  slip_angle_averager( ANGLE_F_BY_FS),
		  nick_angle_averager( ANGLE_F_BY_FS)
		  {
		  }
	void attitude_setup( const float3vector & acceleration, const float3vector & induction);

	void update_compass(
			const float3vector &gyro, const float3vector &acc, const float3vector &mag,
			const float3vector &GNSS_acceleration); //!< rotate quaternion taking angular rate readings

	void update_diff_GNSS( const float3vector &gyro, const float3vector &acc, const float3vector &mag,
		const float3vector &GNSS_acceleration,
		float GNSS_heading); //!< rotate quaternion taking angular rate readings

	inline void set_from_euler( float r, float n, float y)
	{
		attitude.from_euler( r, n, y);
		attitude.get_rotation_matrix( body2nav);
		body2nav.transpose( nav2body);
		euler = attitude;
	}
	inline eulerangle<ftype> get_euler(void) const
	{
		return euler;
	}
	inline const float3vector &get_nav_acceleration(void) const
	{
		return acceleration_nav_frame;
	}
	inline const float3vector &get_nav_induction(void) const
	{
		return induction_nav_frame;
	}
	inline const float get_lin_e0(void) const
	{
		return attitude.lin_e0();
	}
	inline const float get_lin_e1(void) const
	{
		return attitude.lin_e1();
	}
	inline const float get_e2(void) const
	{
		return attitude.get_e2();
	}
	inline const float get_north(void) const
	{
		return attitude.get_north();
	}
	inline const float get_east(void) const
	{
		return attitude.get_east();
	}
	inline const float get_down(void) const
	{
		return attitude.get_down();
	}
	inline const float3vector get_orientation(void) const
	{
	  float3vector retv;
	  retv[0] = get_north();
	  retv[1] = get_east();
	  retv[2] = get_down();
	  return retv;
	}
	inline const float3vector &get_gyro_correction(void) const
	{
		return gyro_correction;
	}
#if 0
	inline const float3vector &get_induction( void) const
	{
		return induction_nav_frame;
	}
#endif
	inline const float3matrix &get_nav2body( void) const
	{
	  return nav2body;
	}
	inline const float3matrix &get_body2nav( void) const
	{
	  return body2nav;
	}
	inline const float3vector &get_nav_correction(void)
	{
	  return nav_correction;
	}
	inline const float3vector &get_gyro_correction(void)
	{
	  return gyro_correction;
	}
	circle_state_t get_circling_state(void) const
	{
	  return circle_state;
	}

  float
  getSlipAngle () const
  {
    return slip_angle_averager.get_output();
  }

  float
  getNickAngle () const
  {
    return nick_angle_averager.get_output();
  }

	quaternion<ftype>attitude;
	float turn_rate;
private:
	circle_state_t circle_state;
	circle_state_t update_circling_state( const float3vector &gyro);
	void update( const float3vector &acc, const float3vector &gyro, const float3vector &mag);
	float3vector nav_correction;
	float3vector gyro_correction;
	float3vector gyro_integrator;
	float3vector acceleration_nav_frame;
	float3vector induction_nav_frame;
	float3matrix body2nav;
	float3matrix nav2body;
	eulerangle<ftype> euler;
	float3vector control_body;
	ftype Ts;
	ftype Ts_div_2;
	unsigned circling_counter;
	pt2<float,float> slip_angle_averager;
	pt2<float,float> nick_angle_averager;
};

#endif /* AHRS_H_ */