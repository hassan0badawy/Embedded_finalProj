/**
 * syscalls.c
 * 
 * Minimal implementation of system calls required by the linker 
 * for bare-metal STM32 builds using the GNU Arm Toolchain.
 */

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/unistd.h>

void _exit(int status) {
    (void)status;
    while (1);
}

int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void) {
    return 1;
}

/* Add other stubs if requested by linker later */
