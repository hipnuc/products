import sys
import os
import click
from utils import check_python_version
from commands.cmd_list import cmd_list
from commands.read_data import cmd_read
from commands.cmd_send import cmd_send

@click.group()
def cli():
    """HiPNUC Python Example"""
    check_python_version()

cli.add_command(cmd_list)
cli.add_command(cmd_read)
cli.add_command(cmd_send)

if __name__ == "__main__":
    cli()
