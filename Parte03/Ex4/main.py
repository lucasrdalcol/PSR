#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
from my_classes import *


# --------------------------------------------------
# A simple python script to make operations with complex numbers using classes.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------

def main():

    # define two complex numbers class objects
    c1 = Complex(r=5, i=3)
    c2 = Complex(r=-2, i=7)

    print('c1: ' + str(c1))
    print('c2: ' + str(c2))

    # Test add returning a new complex number class object.
    c3 = c1.add(c2)
    print('c1 + c2: ' + str(c3))

    # ---------------------------------------------------------------------------

    # define two complex numbers class objects
    c1 = Complex(r=5, i=3)
    c2 = Complex(r=-2, i=7)

    # Test add without returning anything
    c1.add(c2)
    print('c1 + c2: ' + str(c1))

    # ---------------------------------------------------------------------------

    # define two complex numbers class objects
    c1 = Complex(r=5, i=3)
    c2 = Complex(r=-2, i=7)

    # Test multiply returning a new complex number class object.
    c4 = c1.multiply(c2)
    print('c1 * c2: ' + str(c4))

    # ---------------------------------------------------------------------------

    # define two complex numbers class objects
    c1 = Complex(r=5, i=3)
    c2 = Complex(r=-2, i=7)

    # Test multiply without returning anything
    c1.multiply(c2)
    print('c1 * c2: ' + str(c1))

if __name__ == "__main__":
    main()
