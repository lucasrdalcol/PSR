#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
from colorama import Fore, Back, Style
from my_functions import *
import argparse

# --------------------------------------------------
# A simple python script to check if a number if prime.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

# Use argparse to give arguments to the script in terminal.
ap = argparse.ArgumentParser()
ap.add_argument('-mn', '--maximum_number', type=int, default=100, help="Maximum number to test")
ap.add_argument('-p', '--prints', action='store_true', help="Show prints at screen or not")
args = vars(ap.parse_args())

def main():

    # Initialize with a print
    print("Starting to compute prime numbers up to " + str(args['maximum_number']))

    # Cycle all numbers starting from 1 to maximum_number.
    counter = 0

    # Cycle all numbers starting from 1 to maximum_number.
    for i in range(1, args['maximum_number']):
        # Check if the number is prime or not.
        if isPrime(i):
            counter = counter + 1
            if args['prints']:
                print(Fore.RED + Back.YELLOW + Style.DIM + 'Number ' + Fore.LIGHTMAGENTA_EX + Back.CYAN + Style.BRIGHT
                      + str(i) + Fore.RED + Back.YELLOW + Style.DIM + ' is prime.' + Style.RESET_ALL)
        else:
            if args['prints']:
                print('Number ' + str(i) + ' is not prime.')

    print(Fore.BLUE + 'Between 1 and ' + str(args['maximum_number']) + ' we have ' + str(
        counter) + ' prime numbers.' + Style.RESET_ALL)


if __name__ == "__main__":
    main()
