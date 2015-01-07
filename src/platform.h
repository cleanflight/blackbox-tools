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

#ifdef WIN32
	#define snprintf _snprintf
#endif

typedef void*(*threadRoutine_t)(void *data);

void thread_create(thread_t *thread, threadRoutine_t threadFunc, void *data);

void semaphore_create(semaphore_t *sem, int initialCount);
void semaphore_destroy(semaphore_t *sem);
void semaphore_wait(semaphore_t *sem);
void semaphore_signal(semaphore_t *sem);

bool directory_create(const char *name);

#endif
