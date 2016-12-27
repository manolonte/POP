#include <alsa/asoundlib.h>
#include <signal.h>
#include <math.h>


void main(int argc, char **argv) {
    int ingain = atoi(argv[1]);
    volatile char *reg;
    int fd;
    reg=(char *)pop_direct_hw_init(&fd);
    setinputgain(reg,ingain);
}
