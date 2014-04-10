'''Controller for the door troll.'''

import serial
import time
import datetime
import csv

def is_associate_time(today):
    '''Returns whether associates should have access at the given datetime.'''

    if today.weekday() in [5,6]:
        return True
    elif today.weekday() == 0 and today.hour >= 16:
        return True
    else:
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

def log(message):
    f = open("/home/pi/log.txt", "a")
    f.write("[%s] %s\n" % (time.ctime(), message))
    f.close()

class Board(object):
    def __init__(self, port_name='/dev/ttyAMA0', baud_rate=9600):
        self.port = serial.Serial(port_name, baudrate=baud_rate)

    def unlock(self):
        self.port.write("u\n")

        if self.port.readline().strip() == "OK":
            return True
        else:
            return False

    def get_tag(self):
        '''Blocking.  Return ID if one is sent by board, None otherwise.'''
        line = self.port.readline()
        fields = line.strip().split('\t')

        if len(fields) != 2 or fields[0] != "ID":
            return None
        else:
            return fields[1]

class Members(object):
    def __init__(self, filename='/home/pi/members.tsv'):
        f = open(filename, 'rb')
        self.members = [i for i in csv.DictReader(f, delimiter='\t')]

    def get_by_tag(self, tag_id):
        for m in self.members:
            if tag_id == wiegandify(format_id(m['RFID'])):
                return m
        return None

def run():
    b = Board()
    m = Members()

    log("Program starting.")

    while True:
        tag = b.get_tag()
        member = m.get_by_tag(tag)
        today = datetime.datetime.today()

        if member == None:
            log("Could not find member with tag %s." % (tag,))
        elif (int(member['Paid Year']) < today.year) or (int(member['Paid Month']) < today.month):
            log("Refused access to %s because they are not paid for this month." % member['Email'])
        elif (member['Plan'] == 'Core'):
            log("Granted access to %s because they are a paid core member." % member['Email'])
            b.unlock()
        elif (member['Plan'] == 'Associate') and is_associate_time(today):
            log("Granted access to %s because they are a paid associate during allowed hours." % member['Email'])
            b.unlock()
        elif (member['Plan'] == 'Associate') and not is_associate_time(today):
            log("Refused access to %s because although they're a paid associate it's outside allowed hours." % member['Email'])
        else:
            log("Refused access to %s." % member['Email'])


if __name__ == "__main__":
    run()
