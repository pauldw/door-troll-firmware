'''Controller for the door troll.'''

import serial
import datetime

def check_access(tag, today, core_ids, associate_ids):
    '''Returns True if the provided tag should have access, given the
    current datetime, core IDs and associate IDs provided.
    You can generate the datetime using datetime.datetime.today()'''

    allowed_ids = core_ids[:]

    if today.weekday() in [5,6]:
        allowed_ids += associate_ids
    elif today.weekday() == 0 and today.hour >= 16:
        allowed_ids += associate_ids

    for _id in allowed_ids:
        if tag == wiegandify(_id):
            return True

    return False

def wiegandify(_id):
    '''Performs the same 3-byte truncation that your NFC wiegand readers do.'''
    result = _id[4:6] + _id[2:4] + _id[0:2]
    result = "".join([c.capitalize() for c in result])
    result = result.lstrip('0')

    return result

def format_id(_id):
    '''Strips and pads IDs.'''
    stripped = _id.strip()
    padded = "0" * (8 - len(stripped)) + stripped
    capitalized = "".join([c.capitalize() for c in padded])

    return capitalized

class Board(object):
    def __init__(self, port_name='/dev/ttyUSB0', baud_rate=9600):
        self.port = serial.Serial(port_name, baudrate=baud_rate)

    def unlock(self):
        self.port.write("u\n")

        if self.port.readline().strip() == "OK":
            return True
        else:
            return False

    def get_tag(self):
        '''Blocking.  Return ID if one is sent by board, None otherwise.'''
        fields = self.port.readline().strip().split('\t')

        if len(fields) != 2 or fields[0] != "ID":
            return None
        else:
            return fields[1]

def run():
    b = Board()
    core_ids = [format_id(l) for l in open("core_ids.txt").readlines()]
    associate_ids = [format_id(l) for l in open("associate_ids.txt").readlines()]

    while True:
        tag = b.get_tag()
        if check_access(tag, datetime.datetime.today(), core_ids, associate_ids):
            b.unlock()
            print("Unlocked for %s." % tag)
        else:
            print("Refused unlock for %s." % tag)

if __name__ == "__main__":
    run()
