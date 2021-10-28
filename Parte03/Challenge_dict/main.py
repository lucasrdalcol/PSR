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

def add(x, y):
    """
    Function to add the complex number class object to other complex number class object given.
    :param y: Complex number class object
    """

    # Add the real parts and the imaginary parts separately and store in class instances
    real_x = x['r']
    imag_x = x['i']
    real_y = y['r']
    imag_y = y['i']

    real_part = real_x + real_y
    imag_part = imag_x + imag_y

    return {'r': real_part, 'i': imag_part}


def main():

    complex_numbers_dict = {}

    complex_numbers_dict['add'] = lambda x, y: {'r': x['r'] + y['r'], 'i': x['i'] + y['i']}

    complex_numbers_dict['c1'] = {'r': 5, 'i': 3}
    complex_numbers_dict['c2'] = {'r': 3, 'i': -7}

    complex_numbers_dict['c3'] = complex_numbers_dict['add'](complex_numbers_dict['c1'], complex_numbers_dict['c2'])

    print(complex_numbers_dict)
    print(complex_numbers_dict['c1'])
    print(complex_numbers_dict['c2'])
    print(complex_numbers_dict['c3'])

if __name__ == "__main__":
    main()
