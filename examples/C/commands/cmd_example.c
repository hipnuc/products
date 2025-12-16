#include "global_options.h"
#include "example_data.h"

int cmd_example(GlobalOptions *opts, int argc, char *argv[]) {
    (void)opts; (void)argc; (void)argv;
    return process_example_data();
}