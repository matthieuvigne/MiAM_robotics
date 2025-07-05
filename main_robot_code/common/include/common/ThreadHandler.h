#ifndef THREAD_HANDLER_H
#define THREAD_HANDLER_H

#include <mutex>
#include <string>
#include <vector>
#include <thread>

/**
 * The ThreadHandler class defines the `GetInstance` method that serves as an
 * alternative to constructor and lets clients access the same instance of this
 * class over and over.
 */
class ThreadHandler
{

    /**
     * The ThreadHandler's constructor/destructor should always be private to
     * prevent direct construction/desctruction calls with the `new`/`delete`
     * operator.
     */
private:
    std::vector<pthread_t> createdThreads_;

protected:
    ThreadHandler()
    {
    }
    ~ThreadHandler() {}

public:
    /**
     * ThreadHandlers should not be cloneable.
     */
    ThreadHandler(ThreadHandler &other) = delete;
    /**
     * ThreadHandlers should not be assignable.
     */
    void operator=(const ThreadHandler &) = delete;
    /**
     * This is the static method that controls the access to the ThreadHandler
     * instance. On the first run, it creates a ThreadHandler object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */

    static ThreadHandler& GetInstance();
    /**
     * Finally, any ThreadHandler should define some business logic, which can be
     * executed on its instance.
     */

    static void removeAllThreads();
    static pthread_t addThread(std::thread& newThread, bool detach = true);


};

#endif