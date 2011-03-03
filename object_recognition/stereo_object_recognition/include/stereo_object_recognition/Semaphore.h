/* 
 * File:   Semaphore.h
 * Author: gedikli
 *
 * Created on 7. August 2010, 22:29
 */

#ifndef STEREO_OBJECT_RECOGNITION_SEMAPHORE_H
#define	STEREO_OBJECT_RECOGNITION_SEMAPHORE_H
#include "boost/thread.hpp"
#include "boost/noncopyable.hpp"

using boost::mutex;
using boost::condition_variable;

/**
 * \brief this is an implementation of a semaphore based on boost synchronization lib
 * it implements the multiple produces, multiple consumer idiom
 */
class Semaphore : public boost::noncopyable
{
public:
  /**
   * \brief constructor
   * \param[in] initial_count value of semaphore at creation time
   */
  Semaphore( unsigned initial_count = 0 );

  /**
   * \briefvirtual destructor
   */
  virtual ~Semaphore( );

  /**
   * \brief increases the value of the semaphore and wakes up one waiting thread, if there is one.
   */
  void push();

  /**
   * \brief Decreases the value of the semaphore. The calling thread is blocked until the value > 0
   */
  void pop();

  /**
   * \brief non-blocking version of the method pop.
   * \return true if value could be descreased, false otherwise
   */
  bool try_pop();

  /**
   * \brief timed-blocking version of the method pop.
   * \param[in] u_seconds microseconds before time-out occurs
   * \return true if value could be decrease before time-out occurs, false otherwise
   */
  bool timed_pop( unsigned long u_seconds );

  /**
   * \brief returns the current value of the semaphore
   * \return current value of the semaphore
   */
  unsigned long getCount() const;

  /**
   * \brief returns the number of blocked threads
   * \return number of blocked threads
   */
  unsigned getWaitingThreadsCount() const;

protected:
  mutable mutex mutex_;
  condition_variable condition_;
  unsigned long count_;
  unsigned waiting_threads_count;
};

#endif	/* STEREO_OBJECT_RECOGNITION_SEMAPHORE_H */

