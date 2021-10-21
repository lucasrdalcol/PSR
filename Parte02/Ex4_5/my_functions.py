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
    # Convert to decimal representation
    decimal_number = ord(stop_char)
    print('Printing all values up to stop_char ' + stop_char)
    for i in range(ord(' '), decimal_number + 1):
        print('ASCII table - Decimal Representation: ' + str(i) + ' , char representation: ' + chr(i))

def readAllUpTo(stop_key):

    while True:
        print('Type something (press X to stop).')
        pressed_key = readchar.readkey()

        if pressed_key == stop_key:
            print('You typed ' + pressed_key + ', finishing the program.')
            break
        else:
            print('Thanks for typing ' + pressed_key)

def countNumbersUpto(stop_key):
    # Ask for all the entries and put them in a list
    pressed_keys = []  # empty list to start with
    while True:
        print('Type something (X to stop)')
        pressed_key = readchar.readkey()

        if pressed_key == stop_key:
            print('You typed ' + Fore.RED + Style.BRIGHT + pressed_key + Style.RESET_ALL + ' terminating.')
            break
        else:
            # print('Thank you for typing ' + Fore.RED + Style.BRIGHT + pressed_key + Style.RESET_ALL)
            pressed_keys.append(pressed_key)

    print('The keys you pressed are:' + str(pressed_keys))

    # Analyse the list and count
    count_pressed_numbers = 0
    count_pressed_others = 0
    pressed_numbers = []
    pressed_others = []
    pressed_others_dict = {}
    for idx_pressed_key, pressed_key in enumerate(pressed_keys):
        if str.isnumeric(pressed_key):
            count_pressed_numbers += 1
            pressed_numbers.append(pressed_key)
        else:
            count_pressed_others += 1
            pressed_others.append(pressed_key)
            pressed_others_dict[str(idx_pressed_key+1)] = pressed_key

    print('You entered ' + str(count_pressed_numbers) + ' numbers: ' + str(pressed_numbers))
    print('You entered ' + str(count_pressed_others) + ' others:' + str(pressed_others))
    print('You entered ' + str(count_pressed_others) + ' others, and your dict is:' + str(pressed_others_dict))

    # Sorting the numbers of pressed_numbers
    pressed_numbers.sort()
    print('The pressed_numbers list sorted is: ' + str(pressed_numbers))
    # Taking out the numbers that are repeated
    pressed_numbers = list(set(pressed_numbers))
    pressed_numbers.sort()
    print('The pressed_numbers list sorted without repetition is: ' + str(pressed_numbers))

    # Using list comprehension to create again pressed_numbers
    list_comp_pressed_numbers = [pressed_key for pressed_key in pressed_keys if str.isnumeric(pressed_key)]
    print('You entered ' + str(count_pressed_numbers) + ' numbers: ' + str(list_comp_pressed_numbers))


    # Option A: for lists
    colors = list(vars(Fore).values())
    for i, pressed_other_key in enumerate(pressed_others):
        print(colors[i] + pressed_others[i], end='')
    print('')

    # Option B: for dictionaries
    colors = list(vars(Fore).values())
    for pressed_other_dict_key, pressed_other_dict in pressed_others_dict.items():
        print(colors[int(pressed_other_dict_key)] + pressed_other_dict, end='')
    print('')

