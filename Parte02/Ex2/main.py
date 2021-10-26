#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
from colorama import Fore, Back, Style
from my_functions import *

# --------------------------------------------------
# A simple python script to check if a number if prime.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

maximum_number = 100  # Maximum number to test


def main():

    # Initialize with a print
    print("Starting to compute prime numbers up to " + str(maximum_number))

    # Initialize counter for the number of primes.
    counter = 0

    # Cycle all numbers starting from 1 to maximum_number.
    for i in range(1, maximum_number):
        # Check if the number is prime or not.
        if isPrime(i):
            counter = counter + 1
            print(Fore.RED + Back.YELLOW + Style.DIM + 'Number ' + Fore.LIGHTMAGENTA_EX + Back.CYAN + Style.BRIGHT
                  + str(i) + Fore.RED + Back.YELLOW + Style.DIM + ' is prime.' + Style.RESET_ALL)
        else:
            print('Number ' + str(i) + ' is not prime.')

    # Print how many prime numbers we have in that range.
    print(Fore.BLUE + 'Between 1 and ' + str(maximum_number) + ' we have ' + str(
        counter) + ' prime numbers.' + Style.RESET_ALL)


if __name__ == "__main__":
    main()
