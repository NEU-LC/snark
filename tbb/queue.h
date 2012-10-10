#ifndef SNARK_TBB_QUEUE_H_
#define SNARK_TBB_QUEUE_H_

#include <boost/thread/condition.hpp>
#include <tbb/concurrent_queue.h>

namespace snark{ namespace tbb{ 

class counter
{
    public:
        /// constructor
        counter() : value_( 0 ), shutdown_( false ) {}
        
        /// increment
        unsigned int operator++()
        {
             boost::lock_guard< boost::mutex > lock( mutex_ );
             ++value_;
             condition_.notify_all();
             return value_;
        }
        
        /// decrement; block, if 0
        unsigned int operator--()
        {
            boost::unique_lock<boost::mutex> lock( mutex_ );
            while( value_ == 0 && !shutdown_ ) { condition_.timed_wait( lock, boost::posix_time::milliseconds( 100 ) ); }
            if( value_ > 0 ) { --value_; }
            return value_;
        }
        
        /// block until not empty
        unsigned int wait_until_non_zero()
        {
            boost::unique_lock<boost::mutex> lock( mutex_ );
            while( value_ == 0 && !shutdown_ ) { condition_.timed_wait( lock, boost::posix_time::milliseconds( 100 ) ); }
            return value_;
        }
        
        /// unlock, if locked in operator--()
        void shutdown() { shutdown_ = true; condition_.notify_all(); }
        
    private:
        boost::condition_variable condition_;
        boost::mutex mutex_;
        unsigned int value_;
        bool shutdown_;
        
};

/// concurrent queue with wait function
/// @todo use some native mechanism from tbb
///       or at least tbb::vector instead of
///       queue for locking
template< typename T >
class queue
{
public:
    queue() {}
    queue( unsigned int capacity ) { m_queue.set_capacity( capacity ); }
    ~queue() {}

    void push( const T& t ) { m_queue.push( t ); ++counter_; }
    void pop( T& t ) { m_queue.pop( t ); --counter_; }
    unsigned int size() const { return m_queue.size(); }
    bool empty() const { return m_queue.empty(); }
    void wait() { counter_.wait_until_non_zero(); }
    void shutdown() { counter_.shutdown(); }

private:
    ::tbb::concurrent_bounded_queue< T > m_queue;
    counter counter_;

};
    
} } 

#endif // SNARK_TBB_QUEUE_H_
