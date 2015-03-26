#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdbool.h>

#if defined(__APPLE__)
    //MacOS doesn't have POSIX unnamed semaphores. Grand Central Dispatch provides an alternative:
    #include <dispatch/dispatch.h>
#elif defined(WIN32)
    #include <windows.h>
#else
    #include <semaphore.h>
#endif

#ifndef WIN32
    #include <pthread.h>
#endif

#if defined(__APPLE__)
    typedef pthread_t thread_t;
    typedef dispatch_semaphore_t semaphore_t;
#elif defined(WIN32)
    typedef HANDLE thread_t;
    typedef HANDLE semaphore_t;
#else
    typedef pthread_t thread_t;
    typedef sem_t semaphore_t;
#endif

// Support for memory-mapped files:
#ifdef WIN32
    #include <windows.h>
    #include <io.h>
#else
    // Posix-y systems:
    #include <sys/mman.h>
#endif

#ifdef WIN32
    #define snprintf _snprintf
#endif

typedef struct fileMapping_t {
#if defined(WIN32)
    HANDLE mapping;
#endif

    int fd;
    const char *data;
    size_t size;
} fileMapping_t;

typedef void*(*threadRoutine_t)(void *data);

void thread_create_detached(threadRoutine_t threadFunc, void *data);

bool mmap_file(fileMapping_t *mapping, int fd);
void munmap_file(fileMapping_t *mapping);

void semaphore_create(semaphore_t *sem, int initialCount);
void semaphore_destroy(semaphore_t *sem);
void semaphore_wait(semaphore_t *sem);
void semaphore_signal(semaphore_t *sem);

bool directory_create(const char *name);

void platform_init();

#endif
