#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import readchar
from colorama import Fore, Back, Style
from time import time
from collections import namedtuple

# ------------------------
##   DATA STRUCTURES   ##
# ------------------------
Complex = namedtuple('Complex', ['r', 'i'])


# ------------------------
## FUNCTION DEFINITION ##
# ------------------------
def add(x, y):
    """
    Function to add the complex number from a dictionary to other complex number from a dictionary given.
    :param x: Complex number from a dictionary
    :param y: Complex number from a dictionary
    :return: A dictionary with a complex number, the keys are the real part of the imaginary part.
    """

    # Add the real parts and the imaginary parts separately
    real_x, imag_x = x['r'], x['i']
    real_y, imag_y = y['r'], y['i']

    real_part = real_x + real_y
    imag_part = imag_x + imag_y

    return {'r': real_part, 'i': imag_part}


def multiply(x, y):
    """
    Function to multiply the complex number from a dictionary to other complex number from a dictionary given.
    :param x: Complex number from a dictionary
    :param y: Complex number from a dictionary
    :return: A dictionary with a complex number, the keys are the real part of the imaginary part.
        """

    # Option A: without using complex built-in function.
    # Get real and imaginary parts of the complex number given
    real_x, imag_x = x['r'], x['i']
    real_y, imag_y = y['r'], y['i']

    # Multiply each part of the first complex by each part of the second complex number
    firsts = real_x * real_y
    outers = real_x * imag_y
    inners = imag_x * real_y
    lasts = imag_x * imag_y

    # Calculate the real and the imaginary parts
    real_part = firsts - lasts
    imag_part = outers + inners

    return {'r': real_part, 'i': imag_part}

