#ifndef MUTEX_IMPLEMENTATION_H_
#define MUTEX_IMPLEMENTATION_H_

#include "FreeRTOS_wrapper.h"
#include "my_assert.h"
#include "system_configuration.h"

extern Mutex EEPROM_lock;

#if RECURSIVE_LOCKS

class Mutex_Wrapper_Type
{
public:
  Mutex_Wrapper_Type( char * name)
 : lock_count(0),
   EEPROM_lock( name)
  {}

  void lock( void)
  {
    if( 0 == __atomic_fetch_add( &lock_count, 1u, __ATOMIC_RELAXED))
      {
	bool success;
	success = EEPROM_lock.lock( MUTEX_TIMEOUT);
	ASSERT( success);
      }
    else
      asm("bkpt 0"); // todo patch
  }

  void unlock( void)
  {
    ASSERT( lock_count > 0);
    __atomic_fetch_sub(  &lock_count, 1u, __ATOMIC_RELAXED);
    if( lock_count == 0)
      EEPROM_lock.release();
  }
private:
  unsigned lock_count;
  Mutex EEPROM_lock;
};

#else

class Mutex_Wrapper_Type
{
public:
  Mutex_Wrapper_Type( char * name)
 : lock_count(0),
   EEPROM_lock( name)
  {}

  void lock( void)
  {
    bool success = EEPROM_lock.lock(25);
    ASSERT( success);
  }

  void unlock( void)
  {
    EEPROM_lock.release();
  }
private:
  unsigned lock_count;
  Mutex EEPROM_lock;
};

#endif

#include "scoped_lock.h"

extern Mutex_Wrapper_Type my_mutex;
#define LOCK_SECTION() ScopedLock lock( my_mutex)

#endif /* MUTEX_IMPLEMENTATION_H_ */
