/* 
 * File:   SynchronizedQueue.h
 * Author: gedikli
 *
 * Created on 7. August 2010, 23:36
 */

#ifndef STEREO_OBJECT_RECOGNITION_SYNCHRONIZEDQUEUE_H
#define	STEREO_OBJECT_RECOGNITION_SYNCHRONIZEDQUEUE_H
#include <queue>
#include <stereo_object_recognition/Semaphore.h>
#include  <iostream>

using namespace std;

/**
 * \brief thread-safe queue
 */
template<typename T>
class SynchronizedQueue : public boost::noncopyable
{
public:
  /**
   * \brief constructor
   * \param [in] size if size is > 0 the queue has fixed size, otherwise it can take "unlimited" number of objects
   */
  SynchronizedQueue( unsigned size = 0 );

  /**
   * \brief virtual destructor
   */
  virtual ~SynchronizedQueue( );

  /**
   * \brief adds an element in the queue. Calling thread gets blocked if queue has fixed size and queue is full.
   * \param [in] element The element to add to the queue
   */
  void push_back(const T& element );

  /**
   * \brief gets an element from the queue. Calling thread gets blocked until an element is available.
   * \return The front element of the queue.
   */
  T pop_front();

  /**
   * \brief non-blocking version of above method pop_front. Instead it returns true if sucessful or false if queue was empty
   * \param[in] element The front element of the queue.
   * \return true if sucess, false otherwise (queue empty)
   */ 
  bool try_pop_front( T& element );

  /**
   * \brief timed-blocking version of above method pop_front.
   * \param[in] element The front element of the queue.
   * \param[in] u_second time in micro-seconds until time-out occurs
   * \return true if sucess, false otherwise (time-out)
   */
  bool timed_pop_front( T& element, unsigned long u_seconds );

  /**
   * \brief returns the number of available elements in the queue
   * \return number of available elements in the queue
   */
  size_t size() const;

  /**
   * \brief removes all elements from queue
   */
  void clear();

protected:
  /**
   * \brief get element at front of queue and remove it atomically
   * \param[in] element the removed element
   */
  void atomic_pop_front( T& element );

  // queue holding the elements in right order
  std::queue<T> queue_;
  // semaphore for elements in queue
  Semaphore elements_;
  // semaphore for empty slots in queue
  Semaphore slots_;
  // mutex for accessing the queue
  mutable mutex mutex_;
  // size of queue if it has a fixed size, otherwise its 0
  unsigned size_;
};

template<typename T>
SynchronizedQueue<T>::SynchronizedQueue( unsigned size )
  : slots_( size )
  , size_( size )
{
}

template<typename T>
SynchronizedQueue<T>::~SynchronizedQueue( )
{
}

template<typename T>
void SynchronizedQueue<T>::push_back(const T& t)
{
  if( size_ ) // if size > 0 => que with fix length => get one free slot
    slots_.pop();

  mutex::scoped_lock lock( mutex_ );
  queue_.push( t );
  elements_.push();
}

template<typename T>
void SynchronizedQueue<T>::atomic_pop_front( T& t )
{
  mutex::scoped_lock lock( mutex_ );
  t = queue_.front();
  queue_.pop();

  if( size_ ) // if size > 0 => que with fix length => add one free slot
    slots_.push();
}

template<typename T>
T SynchronizedQueue<T>::pop_front()
{
  elements_.pop(); // => blocked here or we have at least one element inside the queue
  T val;
  atomic_pop_front( val );
  return val;
}

template<typename T>
bool SynchronizedQueue<T>::try_pop_front( T& t )
{
  if( elements_.try_pop() )
  {
    atomic_pop_front( t );
    return true;
  }
  return false;
}

template<typename T>
bool SynchronizedQueue<T>::timed_pop_front( T& t, unsigned long u_seconds )
{
  if( elements_.timed_pop( u_seconds ) )
  {
    atomic_pop_front( t );
    return true;
  }
  return false;
}

template<typename T>
size_t SynchronizedQueue<T>::size() const
{
  mutex::scoped_lock lock( mutex_ );
  return queue_.size();
}

template<typename T>
void SynchronizedQueue<T>::clear()
{
  mutex::scoped_lock lock( mutex_ );
  while( !queue_.empty() )
  {
    elements_.pop();
    queue_.pop();
    if( size_ ) // if size > 0 => que with fix length => add one free slot
      slots_.push();
  }
}
#endif	/* STEREO_OBJECT_RECOGNITION_SYNCHRONIZEDQUEUE_H */

