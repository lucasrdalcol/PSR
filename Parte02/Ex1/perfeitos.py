#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
from colorama import Fore, Back, Style

# --------------------------------------------------
# A simple python script to check if a number if perfect.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

maximum_number = 100  # Maximum number to test


def getDividers(value):
    """
    Return a list of dividers for the number value
    :param value: the number to test
    :return: a list of dividers.
    """

    dividers = []

    # Cycle all the integer numbers until value to see if that number is a divider of value
    for i in range(1, value):
        remainder = value % i
        if remainder == 0:
            dividers.append(i)

    return dividers


def isPerfect(value):
    """
    Checks whether the number value is perfect or not.
    :param value: the number to test.
    :return: True or False
    """

    dividers = getDividers(value)
    sum_of_dividers = sum(dividers)
    if sum_of_dividers == value:
        return True
    else:
        return False


def main():

    # Initialize with a print
    print("Starting to compute perfect numbers up to " + str(maximum_number))

    # Cycle all numbers starting from 1 to maximum_number.
    for i in range(1, maximum_number):
        # Check if the number is perfect.
        if isPerfect(i):
            print(Fore.GREEN + Back.BLACK + Style.DIM + 'Number ' + str(i) + ' is perfect.' + Style.RESET_ALL)
        else:
            print(Fore.RED + Back.BLACK + Style.DIM + 'Number ' + str(i) + ' is not perfect.' + Style.RESET_ALL)


if __name__ == "__main__":
    main()
