/*

The MIT License (MIT)

Copyright (c) 2016 rklinkhammer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef THREADED_QUEUE_H_
#define THREADED_QUEUE_H_

#include <queue>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace rrcs {

/**
* @brief Thread-safe FIFO queue with blocking, signalling, and cleanup.
*
* ThreadedQueue is a template class that is designed to manage the enqueuing
* and dequeuing of elements, blocking on the queue when empty.  It utilizes
* Boost condition variables to manage a count of elements (pending_) and
* an exit flag.
*
* Threads use the Enqueue() and Dequeue() commands under normal circumstances.
* When the Queue is to be shutdown, it must first be disabled.  This can be
* done by calling the DisableQueue() method.  This will set the exit flag and
* wakeup any blocked threads.  The setting of the exit flag will also prevent
* any thread from retrieving or blocking on the queue subsequently.  In order
* to empty the queue itself, there is a method called  DequeueNonBlocking().
* The queue can be re-enabled using the EnableQueue() method.
*
* The following is example code of how to handle disabling a queue and
* emptying the elements.
*
* @code
*        the_queue.DisableQueue();
*        Request request;
*        while(the_queue.DequeueNonBlocking(request)) {
*           Free(request);
*        }
* @endcode
*/
template<class Element> class ThreadedQueue {

public:
    ThreadedQueue(const int depth = 1) : depth_(depth) {
    }

    virtual ~ThreadedQueue() {
    }

    /**
     * @brief Enqueue an element
     *
     * The caller will insert the element into the queue and notify
     * any waiters who may be blocked in Dequeue.
     *
     * @param element - reference to item to enqueue.
     *
     * @return The function will return <em>true</em> if element has been
     *         successfully enqueued; otherwise, the function will return
     *         <em>false</em> if disabled.
     */
    bool Enqueue(const Element& element) {
        // increment condition variable
        std::unique_lock<std::mutex> lock(pending_mutex_);
        if (!exit_) {
            queue_.push(std::move(element));
            pending_++;
            condition_.notify_one();
            return true;
        }
        return false;
    }

    /**
     * @brief Nonblocking Dequeue from queue
     *
     * This method is intended to be used to empty the queue.  Especially
     * during cleanup.
     *
     * @param element - reference to store dequeued item.
     *
     * @return The function will return <em>true</em> if element has been
     *         successfully dequeued; otherwise, the function will return
     *         <em>false</em> if there is nothing left to dequeue.
     */
    bool DequeueNonBlocking(Element& element) {
        std::unique_lock<std::mutex> lock(pending_mutex_);
        if (!pending_) {
            return false;
        }
        pending_--;
        element = queue_.front();
        queue_.pop();
        return true;
    }

    /**
     * @brief Block until element available.
     *
     * The caller will block until an element has been enqueued or
     * until the queue has been signaled via DisableQueue().  If a queue
     * has been signaled, this takes priority over any queued elements.
     *
     * @param element - reference to store dequeued item.
     *
     * @return The function will return <em>true</em> if element has been
     *         successfully dequeued; otherwise, the function will return
     *         <em>false</em> if disabled.

     */
    bool Dequeue(Element& element) {
        std::unique_lock<std::mutex> lock(pending_mutex_);
        while (!exit_ && !pending_) {
            condition_.wait(lock);
        }
        if (exit_) {
            return false;
        }
        pending_--;
        element = queue_.front();
        queue_.pop();
        return true;
    }

    /**
     * @brief Disable ability to enqueue and dequeue elements.
     *
     * This method will set the exit flag and notify a single
     * waiter.  This will not effect any pending queue elements.
     */
    void Disable() {
        // reset condition variable
        std::unique_lock<std::mutex> lock(pending_mutex_);
        exit_ = true;
        condition_.notify_all();
    }

    /**
     * @brief Restore  ability to enqueue and dequeue elements.
     *
     * This method will clear the exit flag and allow waiters
     * to retrieve elements.
     */
    void Enable() {
        // set condition variable
        std::unique_lock<std::mutex> lock(pending_mutex_);
        exit_ = false;
    }

private:
    ThreadedQueue(const ThreadedQueue &theThreadedQueue);
    std::condition_variable condition_; /*!< condition variable */
    std::mutex pending_mutex_; /*!< mutex variable */
    int pending_ {false}; /*!< count of queued elements */
    std::size_t depth_ {0}; /*!< max queue depth */
    std::queue<Element> queue_; /*!< queue */
    bool exit_ {false}; /*!< exit flag */
};

} // namespace rrcs

#endif /* THREADED_QUEUE_H_ */
