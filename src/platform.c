#include "platform.h"

#ifdef WIN32
	#include <direct.h>
#else
	#include <sys/stat.h>
#endif


#ifdef WIN32
	/*
	 * I don't want to have to define my thread routine stdcall on windows and cdecl on POSIX, so
	 * this structure'll wrap things for me
	 */
	typedef struct win32ThreadFuncWrapper_t {
		threadRoutine_t threadFunc;
		void *data;
	} win32ThreadFuncWrapper_t;

	DWORD WINAPI win32ThreadFuncUnwrap(LPVOID data)
	{
		win32ThreadFuncWrapper_t *unwrapped = (win32ThreadFuncWrapper_t*)data;
		DWORD result;

		//Call the original thread routine with the original data and return that as our result
		result = (DWORD) (unwrapped->threadFunc(unwrapped->data));

		free(unwrapped);

		return result;
	}
#endif

void thread_create(thread_t *thread, threadRoutine_t threadFunc, void *data)
{
#if defined(WIN32)
	win32ThreadFuncWrapper_t *wrap = malloc(sizeof(*wrap));

	wrap->threadFunc = threadFunc;
	wrap->data = data;

	*thread = CreateThread(NULL, 0, win32ThreadFuncUnwrap, wrap, 0, NULL);
#else
    pthread_create(thread, NULL, threadFunc, data);
#endif
}

void semaphore_signal(semaphore_t *sem)
{
#if defined(__APPLE__)
	dispatch_semaphore_signal(*sem);
#elif defined(WIN32)
	ReleaseSemaphore(*sem, 1, NULL);
#else
	sem_post(sem);
#endif
}

void semaphore_wait(semaphore_t *sem)
{
#if defined(__APPLE__)
	dispatch_semaphore_wait(*sem, DISPATCH_TIME_FOREVER);
#elif defined(WIN32)
	WaitForSingleObject(*sem, INFINITE);
#else
	sem_wait(sem);
#endif
}

void semaphore_create(semaphore_t *sem, int initialCount)
{
#if defined(__APPLE__)
	*sem = dispatch_semaphore_create(initialCount);
#elif defined(WIN32)
	*sem = CreateSemaphore(NULL, initialCount, initialCount, NULL);
#else
	sem_init(sem, 0, initialCount);
#endif
}

void semaphore_destroy(semaphore_t *sem)
{
#if defined(__APPLE__)
	dispatch_release(*sem);
#elif defined(WIN32)
	CloseHandle(*sem);
#else
sem_destroy(sem);
#endif
}

bool directory_create(const char *name)
{
#if defined(WIN32)
	return _mkdir(name) == 0;
#else
	return mkdir(name, S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IROTH) == 0;
#endif
}
