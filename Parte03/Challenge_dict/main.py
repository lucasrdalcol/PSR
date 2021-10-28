#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
from my_functions import *


# --------------------------------------------------
# A simple python script to make operations with complex numbers using dictionaries.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------


def main():
    # # Option A: Adding inside the dictionary the add function from my_functions
    # # Initialize dictionary for all complex numbers
    # complex_numbers_dict = {}
    #
    # complex_numbers_dict['add'] = add
    # complex_numbers_dict['multiply'] = multiply
    #
    # # Add other dictionary inside the highest level one with each of the complex numbers. keys for real part and
    # # imaginary part
    # complex_numbers_dict['c1'] = {'r': 5, 'i': 3}
    # complex_numbers_dict['c2'] = {'r': -2, 'i': 7}
    #
    # # Use functions inside the dictionary to make the operation.
    # complex_numbers_dict['c3'] = complex_numbers_dict['add'](complex_numbers_dict['c1'], complex_numbers_dict['c2'])
    # complex_numbers_dict['c4'] = complex_numbers_dict['multiply'](complex_numbers_dict['c1'], complex_numbers_dict['c2'])
    #
    # # Print the dictionary and each of the complex number inside it
    # print('My dict is: ' + str(complex_numbers_dict))
    # print('c1 = ' + str(complex_numbers_dict['c1']))
    # print('c2 = ' + str(complex_numbers_dict['c2']))
    # print('c1 + c2 = ' + str(complex_numbers_dict['c3']))
    # print('c1 * c2 = ' + str(complex_numbers_dict['c4']))

    # ----------------------------------------------------------------------------------------------------------
    # Option B: Adding the add function using lambda function (anonymous function), much more pythonic and uses only
    # one line.
    # Initialize dictionary for all complex numbers
    complex_numbers_dict = {}

    complex_numbers_dict['add'] = lambda x, y: {'r': x['r'] + y['r'], 'i': x['i'] + y['i']}
    complex_numbers_dict['multiply'] = lambda x, y: {'r': (x['r'] * y['r']) - (x['i'] * y['i']),
                                                     'i': (x['r'] * y['i']) + (x['i'] * y['r'])}

    # Add other dictionary inside the highest level one with each of the complex numbers. keys for real part and
    # imaginary part
    complex_numbers_dict['c1'] = {'r': 5, 'i': 3}
    complex_numbers_dict['c2'] = {'r': -2, 'i': 7}

    # Use function inside the dictionary to make the operation.
    complex_numbers_dict['c3'] = complex_numbers_dict['add'](complex_numbers_dict['c1'], complex_numbers_dict['c2'])
    complex_numbers_dict['c4'] = complex_numbers_dict['multiply'](complex_numbers_dict['c1'],
                                                                  complex_numbers_dict['c2'])

    # Print the dictionary and each of the complex number inside it
    print('My dict is: ' + str(complex_numbers_dict))
    print('c1 = ' + str(complex_numbers_dict['c1']))
    print('c2 = ' + str(complex_numbers_dict['c2']))
    print('c1 + c2 = ' + str(complex_numbers_dict['c3']))
    print('c1 * c2 = ' + str(complex_numbers_dict['c4']))


if __name__ == "__main__":
    main()
