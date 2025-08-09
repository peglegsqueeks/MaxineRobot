from threading import Event, enumerate, main_thread

# this event handles all the looping threads.
# when it is set, all looping threads will stop what they are doing
THREAD_STOP_EVENT = Event()


def stop_all_threads():
    """
    Sets the event to stop all threads
    """
    THREAD_STOP_EVENT.set()


def run_all_threads():
    """
    Clears the event to run all threads
    """
    THREAD_STOP_EVENT.clear()


def graceful_thread_exit(function):
    """
    Function decorator to handle exiting with multiple threads.
    When the program exits (either because of an exception or exiting), it will:
        - Set the event so all looping threads stop
        - Await all the running threads
    """

    def inner(*args, **kwargs):
        try:
            function(*args, **kwargs)
        finally:
            # set event to stop looping threads
            stop_all_threads()

            # join all threads except for main thread
            main = main_thread()
            for thread in enumerate():
                if thread != main:
                    print(f"awaiting thread {str(thread.__class__.__name__)}")
                    thread.join(timeout=5)

                    # check timeout occured when joining the thread
                    if thread.is_alive():
                        print(f"Error joining thread {str(thread.__class__.__name__)}")

    return inner
