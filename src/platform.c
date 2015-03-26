#include "platform.h"

#ifdef WIN32
    #include <direct.h>
#else
    #include <sys/stat.h>
#endif

#include <sys/stat.h>

#ifndef WIN32
    #define POSIX
#endif

#ifdef POSIX
    pthread_attr_t pthreadCreateDetached;
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

void thread_create_detached(threadRoutine_t threadFunc, void *data)
{
    thread_t thread;

#if defined(WIN32)
    win32ThreadFuncWrapper_t *wrap = malloc(sizeof(*wrap));

    wrap->threadFunc = threadFunc;
    wrap->data = data;

    thread = CreateThread(NULL, 0, win32ThreadFuncUnwrap, wrap, 0, NULL);

    // Detach from thread immediately
    CloseHandle(thread);
#else
    pthread_create(&thread, &pthreadCreateDetached, threadFunc, data);
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

/**
 * Map the open file with the given file handle `fd` into memory. Store the details about the mapping into `mapping`.
 *
 * Returns true on success
 */
bool mmap_file(fileMapping_t *mapping, int fd)
{
    struct stat stats;

    //Need the file size to complete the mapping
    if (fd < 0 || fstat(fd, &stats) < 0) {
        return 0;
    }

    mapping->fd = fd;
    mapping->size = stats.st_size;

    // The APIs don't like mapping a file of size zero
    if (mapping->size > 0) {
        #ifdef WIN32
            intptr_t fileHandle = _get_osfhandle(fd);
            mapping->mapping = CreateFileMapping((HANDLE) fileHandle, NULL, PAGE_READONLY, 0, 0, NULL);

            if (mapping->mapping == NULL) {
                return false;
            }

            mapping->data = MapViewOfFile(mapping->mapping, FILE_MAP_READ, 0, 0, mapping->size);

            if (mapping->data == NULL) {
                CloseHandle(mapping->mapping);
                return false;
            }
        #else
            mapping->data = mmap(0, mapping->size, PROT_READ, MAP_PRIVATE, fd, 0);

            if (mapping->data == MAP_FAILED) {
                return false;
            }
        #endif
    } else {
        mapping->data = 0;
    }

    return true;
}

void munmap_file(fileMapping_t *mapping)
{
    if (mapping->data) {
        #ifdef WIN32
            UnmapViewOfFile(mapping->data);
            CloseHandle(mapping->mapping);
        #else
            munmap((void*)mapping->data, mapping->size);
        #endif
    }
}

/**
 * Call before any other routines in this unit.
 */
void platform_init()
{
#ifdef POSIX
    pthread_attr_init(&pthreadCreateDetached);
    pthread_attr_setdetachstate(&pthreadCreateDetached, PTHREAD_CREATE_DETACHED);
#endif
}
