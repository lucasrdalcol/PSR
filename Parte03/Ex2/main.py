#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
from my_functions import *


# --------------------------------------------------
# A simple python script to create and operate complex numbers using tuples.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------


def main():

    # define two complex numbers as tuples of size two
    c1 = (5, 3)
    c2 = (-2, 7)
    printComplex(c1, 'c1 = ')
    printComplex(c2, 'c2 = ')

    # Test addComplex function
    c3 = addComplex(c1, c2)
    printComplex(c3, 'c1 + c2 = ')

    # Test multiplyComplex function
    c4 = multiplyComplex(c1, c2)
    printComplex(c4, 'c1 * c2 = ')


if __name__ == "__main__":
    main()
