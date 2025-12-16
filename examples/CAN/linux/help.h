#ifndef HELP_H
#define HELP_H

void help_print_version(void);
void help_print_usage(const char *program_name);
void help_print_unknown_command(const char *program_name);
void help_print_can_setup(const char *ifname);
void help_print_no_interfaces_hint(void);

#endif
