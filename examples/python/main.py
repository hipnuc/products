import sys
import os
import click
from utils import check_python_version
from commands.cmd_list import cmd_list, cmd_probe
from commands.read_data import cmd_read
from commands.cmd_send import cmd_write

@click.group()
def cli():
    """HiPNUC Python Example"""
    check_python_version()

cli.add_command(cmd_list)
cli.add_command(cmd_probe)
cli.add_command(cmd_read)
cli.add_command(cmd_write)
cli.add_command(cmd_write, name="send")

if __name__ == "__main__":
    cli()
