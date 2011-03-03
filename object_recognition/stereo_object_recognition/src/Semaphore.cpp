/* 
 * File:   Semaphore.cpp
 * Author: gedikli
 * 
 * Created on 7. August 2010, 22:29
 */

#include <boost/thread/pthread/mutex.hpp>
#include <boost/thread/pthread/condition_variable_fwd.hpp>

#include "stereo_object_recognition/Semaphore.h"

using boost::lock_guard;
using boost::xtime;
using boost::xtime_get;
using boost::unique_lock;

Semaphore::Semaphore( unsigned initial_count )
 : count_( initial_count )
 , waiting_threads_count( 0 )
{
}

Semaphore::~Semaphore( )
{
}

void Semaphore::push()
{
  // one thread can push at once!
  mutex::scoped_lock lock( mutex_ );
  ++count_;
  condition_.notify_one();
}

void Semaphore::pop()
{
  unique_lock<mutex> lock( mutex_ );

  // we use 'do-while' since it may be, that one calls push, a waiting thread A wakes up but before the mutex could be
  // acquired another non-waiting thread B calls pop => after A gets the mutex, count_ is again 0!
  if( count_ == 0 )
  {
    ++waiting_threads_count;
    do {
      condition_.wait( lock ); // mutex will be unlocked if going to sleep... and atomically be locked at wakeup!
    } while( count_ == 0 );
    --waiting_threads_count;
  }
  --count_;
}

bool Semaphore::try_pop()
{
  mutex::scoped_lock lock( mutex_ );
  if( count_ > 0 )
  {
    --count_;
    return true;
  }
  return false;
}

bool Semaphore::timed_pop( unsigned long u_seconds )
{
  unique_lock<mutex> lock( mutex_ );
  
  if( count_ == 0 )
  {
    xtime end_time;
    xtime_get(&end_time, boost::TIME_UTC);
    end_time.nsec += u_seconds * 1000;

    ++waiting_threads_count;
    bool result;
    do {
      result = condition_.timed_wait( lock, end_time ); // mutex will be unlocked if going to sleep... and atomically be locked at wakeup!
    } while( result && count_ == 0 );
    // repeat until we could acquire the mutex and count > 0 or time out occurs!
    --waiting_threads_count;
  }
  // either time-out or count > 0. In both cases the mutex is locked by the current thread.
  // => we check just for count, since after time-out but before acquiring the mutex some thread may call push
  if( count_ > 0 )
  {
    --count_;
    return true;
  }

  return false;
 }

 unsigned long Semaphore::getCount() const
 {
   unique_lock<mutex> lock( mutex_ );
   return count_;
 }
 
 unsigned Semaphore::getWaitingThreadsCount() const
 {
   unique_lock<mutex> lock( mutex_ );
   return waiting_threads_count;
 }