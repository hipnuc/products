#include "cli.h"
#include <signal.h>
#include <stdio.h>

volatile sig_atomic_t canhost_stop = 0;

static void stop(int signal_number)
{
    (void)signal_number;
    canhost_stop = 1;
}

int main(int argc, char **argv)
{
    canhost_options_t options;
    int parsed = canhost_parse(argc, argv, &options);
    if (parsed > 0) { canhost_help(); return ferror(stdout) ? 1 : 0; }
    if (parsed < 0) return 2;
    signal(SIGINT, stop);
    signal(SIGTERM, stop);
    signal(SIGPIPE, SIG_IGN);  /* turn a closed output pipe into a reported write error */
    int result = canhost_run(&options);
    if (fflush(stdout) != 0) return 1;
    return result;
}
