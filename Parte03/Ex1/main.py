#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import readchar
from colorama import Fore, Back, Style
from my_functions import *
from time import time, ctime
import math


# --------------------------------------------------
# A simple python script to use timers and built-in library time. Also used to print elapse time.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------


def main():

    # Get current date and print it using colorama.
    current_date = ctime()
    print('This is ' + Fore.RED + 'Ex1 ' + Fore.RESET + 'and the current date is ' + Fore.BLUE + Back.LIGHTYELLOW_EX
          + current_date + Style.RESET_ALL)

    # Make a cycle to calculate the elapse time.
    maximum_number = int(50e6)

    tic()
    for i in range(0, maximum_number):
        math.sqrt(i)

    toc()

if __name__ == "__main__":
    main()
