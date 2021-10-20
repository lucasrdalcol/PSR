#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import readchar


# --------------------------------------------------
# A simple python script to check if a number if prime.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------


def printAllCharsUpTo(stop_char):
    # Convert to decimal representation
    decimal_number = ord(stop_char)
    for i in range(32, decimal_number + 1):
        print('ASCII table - Decimal Representation: ' + str(i) + ' , char representation: ' + chr(i))


def main():

    print("Press any key of the keyboard.")

    # Read a key from keyboard
    stop_char = readchar.readkey()
    printAllCharsUpTo(stop_char)

if __name__ == "__main__":
    main()
