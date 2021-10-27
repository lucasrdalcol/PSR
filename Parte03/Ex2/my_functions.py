#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------


# ------------------------
## FUNCTION DEFINITION ##
# ------------------------
def addComplex(x, y):
    """
    Add complex number x and y and return it.
    :param x: A complex number given in as a tuple of size two. The first element is the real part and the second is
              the imaginary part.
    :param y: A complex number given in as a tuple of size two. The first element is the real part and the second is
              the imaginary part.
    :return: real and imaginary parts.
    """

    # Option A: without using complex built-in function.
    # Get real and imaginary parts of the complex number given
    real_x, imag_x = x
    real_y, imag_y = y

    # Add the real parts and the imaginary parts separately
    real_part = real_x + real_y
    imag_part = imag_x + imag_y
    return real_part, imag_part

    # # Option B: using complex built-in function
    # # Convert x and y to complex numbers
    # x = complex(x[0], x[1])
    # y = complex(y[0], y[1])
    #
    # # Add the complex numbers.
    # result = x + y
    # real_part = result.real
    # imag_part = result.imag
    #
    # return real_part, imag_part


def multiplyComplex(x, y):
    """
    Multiply complex number x and y and return it.
    :param x: A complex number given in as a tuple of size two. The first element is the real part and the second is
              the imaginary part.
    :param y: A complex number given in as a tuple of size two. The first element is the real part and the second is
              the imaginary part.
    :return: real and imaginary parts.
    """

    # Option A: without using complex built-in function.
    # Get real and imaginary parts of the complex number given
    real_x, imag_x = x
    real_y, imag_y = y

    # Multiply each part of the first complex by each part of the second complex number
    firsts = real_x * real_y
    outers = real_x * imag_y
    inners = imag_x * real_y
    lasts = imag_x * imag_y

    # Calculate the real and the imaginary parts
    real_part = firsts - lasts
    imag_part = outers + inners

    return real_part, imag_part

    # # Option B: using complex built-in function
    # # Convert x and y to complex numbers
    # x = complex(x[0], x[1])
    # y = complex(y[0], y[1])
    #
    # # Multiply the complex numbers.
    # result = x * y
    # real_part = result.real
    # imag_part = result.imag
    #
    # return real_part, imag_part


def printComplex(x):
    """
    Print a complex numbers
    :param x: A complex number given in as a tuple of size two. The first element is the real part and the second is
              the imaginary part.
    """

    # Get real and imaginary parts of the complex number given
    real_x, imag_x = x

    print(str(real_x) + ' + ' + str(imag_x) + 'i')
