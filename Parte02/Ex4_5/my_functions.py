#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import readchar
from colorama import Fore, Back, Style


# --------------------------------------------------
# A simple python script to check if a number if prime.
# Lucas Rodrigues Dal'Col
# PSR, October 2021.
# --------------------------------------------------


def printAllCharsUpTo(stop_char):
    """
    Function that print all chars up to stop_char, starting at 'space' to stop_char.
    ASCII Table: https://www.asciitable.com/
    :param stop_char: the char/key given by the keyboard.
    """

    # Convert to decimal representation
    decimal_number = ord(stop_char)

    # Initialization print
    print('Printing all values up to stop_char ' + stop_char)

    # Cycle all chars from 'space' to stop_char to print it.
    for i in range(ord(' '), decimal_number + 1):
        print('ASCII table - Decimal Representation: ' + str(i) + ' , char representation: ' + chr(i))


def readAllUpTo(stop_key='X'):
    """
    Read continuously a key from keyboard until 'X' (stop_key) is typed.
    The default stop_key is 'X' but you can change it if you want.
    :param stop_key: char/key given by keyboard to stop function. Default is 'X'.
    """

    # Read continuously.
    while True:
        print('Type something (press ' + Fore.RED + 'X' + Fore.RESET + ' to stop).')
        # Read typed key from keyboard.
        pressed_key = readchar.readkey()

        # Analyse the typed key.
        if pressed_key == stop_key:
            print(
                Back.YELLOW + 'You typed ' + Fore.RED + pressed_key + Fore.RESET + ', finishing the program.' + Style.RESET_ALL)
            break
        else:
            print('Thanks for typing ' + Fore.GREEN + pressed_key + Style.RESET_ALL)


def countNumbersUpto(stop_key='X'):
    """
    Read continuously a key from keyboard until 'X' (stop_key) is typed, then how many chars are algarisms and how
    many are non algarisms. The default stop_key is 'X' but you can change it if you want.
    :param stop_key: char/key given by keyboard to stop function. Default is 'X'.
    """

    # Initialize list of pressed keys.
    pressed_keys = []  # empty list to start with

    # Read continuously.
    while True:
        print('Type something (press ' + Fore.RED + 'X' + Fore.RESET + ' to stop).')
        # Read typed key from keyboard.
        pressed_key = readchar.readkey()

        # Analyse the typed key.
        if pressed_key == stop_key:
            print('You typed ' + Fore.RED + Style.BRIGHT + pressed_key + Style.RESET_ALL + ' terminating.')
            break
        else:
            print('Thank you for typing ' + Fore.GREEN + Style.BRIGHT + pressed_key + Style.RESET_ALL)
            pressed_keys.append(pressed_key)

    print('The keys you pressed are: ' + str(pressed_keys))

    # Analyse the list and count.
    count_pressed_numbers = 0
    count_pressed_others = 0
    pressed_numbers = []
    pressed_others = []

    # Exercise 5: Using dictionaries.
    pressed_others_dict = {}

    # Cycle the list.
    for idx_pressed_key, pressed_key in enumerate(pressed_keys):
        # Append the list of numbers and the list of others
        if str.isnumeric(pressed_key):
            count_pressed_numbers += 1
            pressed_numbers.append(pressed_key)
        else:
            count_pressed_others += 1
            pressed_others.append(pressed_key)

            # Exercise 5: Create the dictionary key with the order that the key was inputted
            pressed_others_dict[str(idx_pressed_key + 1)] = pressed_key

    # Print the counting.
    print('You entered ' + str(count_pressed_numbers) + ' numbers: ' + str(pressed_numbers))
    print('You entered ' + str(count_pressed_others) + ' others:' + str(pressed_others))
    print('You entered ' + str(count_pressed_others) + ' others, and your dict is:' + str(pressed_others_dict))

    # 5d) Sorting the numbers of pressed_numbers
    pressed_numbers.sort()
    print('The pressed_numbers list sorted is: ' + str(pressed_numbers))

    # Taking out the numbers that are repeated
    pressed_numbers = list(set(pressed_numbers))
    pressed_numbers.sort()  # After set it's needed to sort again.
    print('The pressed_numbers list sorted without repetition is: ' + str(pressed_numbers))

    # 5e) Using list comprehension to create again pressed_numbers. Other way to do Exercise 5b
    list_comp_pressed_numbers = [pressed_key for pressed_key in pressed_keys if str.isnumeric(pressed_key)]
    print('You entered ' + str(count_pressed_numbers) + ' numbers, your list using list comprehension is: '
          + str(list_comp_pressed_numbers))


    # Challenge 1: Print in the same line of the terminal the elements of the pressed_others list and change the
    # color for each element

    # Option A: for lists
    colors = list(vars(Fore).values())*10
    for i, pressed_other_key in enumerate(pressed_others):
        print(colors[i] + pressed_others[i], end='')
    print('')

    # Option B: for dictionaries
    for pressed_other_dict_key, pressed_other_dict in pressed_others_dict.items():
        print(colors[int(pressed_other_dict_key)] + pressed_other_dict, end='')
    print('')
