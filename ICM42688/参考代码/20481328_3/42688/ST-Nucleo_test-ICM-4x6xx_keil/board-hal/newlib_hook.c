/*
 * newlib_stubs.c
 *
 *  Created on: 2 Nov 2010
 *      Author: nanoage.co.uk
 */
#include <errno.h>

#ifndef __ARMCC_VERSION

#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#include "stm32f4xx.h"

#undef errno
extern int errno;

/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

int _write(int file, char *ptr, int len);

void _exit(int status)
{
	(void)status;
    _write(1, "exit", 4);

	for( ; ; )
		;
}

int _open(const char *name, int flags, int mode)
{
	(void)name, (void)flags, (void)mode;
	return -1;
}

int _close(int file)
{
	(void)file;
    return -1;
}
/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env)
{
	(void)name, (void)argv, (void)env;
    errno = ENOMEM;
    return -1;
}
/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */

int _fork()
{
    errno = EAGAIN;
    return -1;
}
/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st)
{
	(void)file, (void)stat;
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */

int _getpid()
{
    return 1;
}

/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file)
{
    switch (file)
	{
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}


/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig)
{
	(void)pid, (void)sig;
    errno = EINVAL;
    return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */

int _link(char *old, char *new)
{
	(void)old, (void)new;
    errno = EMLINK;
    return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir)
{
	(void)file, (void)ptr, (void)dir;
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr)
{
    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

	if (heap_end == 0) {
		heap_end = &_ebss;
	}
	prev_heap_end = heap_end;

	char * stack = (char*) __get_MSP();
	if (heap_end + incr >  stack)
	{
		_write (STDERR_FILENO, "Heap and stack collision\n", 25);
		errno = ENOMEM;
		return  (caddr_t) -1;
	}

	heap_end += incr;
	return (caddr_t) prev_heap_end;
}

/*
 read
 Read a character to a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */

int _read(int file, char *ptr, int len)
{
    int n;
    int num = 0;
    switch (file) {
    case STDIN_FILENO:
        for (n = 0; n < len; n++)
		{
			while ((USART6->SR & USART_FLAG_RXNE) == (uint16_t)RESET) ;
            char c = (char)(USART6->DR & (uint16_t)0x01FF);
			*ptr++ = c;
            num++;
        }
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return num;
}

/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */

int _stat(const char *filepath, struct stat *st)
{
	(void)filepath, (void)st;
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */

clock_t _times(struct tms *buf)
{
	(void)buf;
    return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name)
{
	(void)name;
    errno = ENOENT;
    return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status)
{
	(void)status;
    errno = ECHILD;
    return -1;
}

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, char *ptr, int len)
{
    int n;
    switch (file)
	{
    case STDOUT_FILENO: case STDERR_FILENO: 
        for (n = 0; n < len; n++)
		{
            while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) {}
            USART_SendData(USART6, *(ptr++));
        }
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}

#endif
