#!/usr/bin/python3

# --------------------------------------------------
# IMPORT MODULES
# --------------------------------------------------
import socket
import time

# --------------------------------------------------
# Miguel Riem Oliveira.
# PSR, September 2020.
# Adapted from https://stackabuse.com/basic-socket-programming-in-python/
# --------------------------------------------------
from dog_lib import Dog


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # create TCP/IP socket
    local_hostname = socket.gethostname()  # retrieve local hostname
    local_fqdn = socket.getfqdn()  # get fully qualified hostname
    ip_address = socket.gethostbyname(local_hostname)  # get the according IP address

    server_address = (ip_address, 23456)  # bind the socket to the port 23456, and connect
    sock.connect(server_address)
    print("connecting to %s (%s) with %s" % (local_hostname, local_fqdn, ip_address))

    # # define example data to be sent to the server
    # messages = [30, 'Robotics', 31, 14, 'Automation', 18]

    # Create a dog instance to transmit to the server
    dog = Dog(name='Boby', color='brown', age=7)
    dog.addBrother('Lassie')
    dog.addBrother('Max')
    dog.addBrother('Marley')

    # Create messages by marshaling/serializing dog to a list
    messages = [dog.name, ',', dog.color, ',', str(dog.age)]
    for brother in dog.brothers:
        messages.append(',')
        messages.append(brother)

    text_to_send = ''.join(messages)

    message_formated = text_to_send.encode('utf-8')
    sock.sendall(message_formated)
    time.sleep(2)  # wait for two seconds

    sock.close()  # close connection


if __name__ == "__main__":
    main()
