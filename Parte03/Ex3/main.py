#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import readchar
import colorama
from my_functions import *
from time import time, ctime
import math
from collections import namedtuple

# ------------------------
##   DATA STRUCTURES   ##
# ------------------------
Complex = namedtuple('Complex', ['r', 'i'])


# --------------------------------------------------
# A simple python script to make operations with complex numbers using namedtuples.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

def main():
    # define two complex numbers as namedtuple of size two
    c1 = Complex(r=5, i=3)
    c2 = Complex(r=-2, i=7)

    # See how namedtuple Complex prints
    print('namedtuple c1 = ' + str(c1))
    print('namedtuple c2 = ' + str(c2))

    # Test addComplex function
    c3 = addComplex(c1, c2)
    printComplex(c3, 'c1 + c2 = ')

    # Test multiplyComplex function
    c4 = multiplyComplex(c1, c2)
    printComplex(c4, 'c1 * c2 = ')


if __name__ == "__main__":
    main()
