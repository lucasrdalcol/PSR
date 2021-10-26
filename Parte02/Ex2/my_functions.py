#!/usr/bin/python3

# --------------------------------------------------
# My functions of Exercise 2 of Parte02
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------


def isPrime(value):
    """
    Checks whether the number value is prime or not.
    :param value: the number to test.
    :return: True or False
    """
    print('\nReference number ' + str(value))

    for i in range(2, value):
        remainder = value % i
        print(str(value) + ' / ' + str(i) + ' has remainder ' + str(remainder))
        if remainder == 0:
            return False

    return True
