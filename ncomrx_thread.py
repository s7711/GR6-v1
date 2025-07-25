# ncomrx_thread.py
# Licensed under the MIT License – see LICENSE file for details.


"""
ncomrx_thread.py

Sets up a background thread, which receives data from OxTS INSs
(on port 3000). Each IP address is send to a separate NComRx decoder.

Use by:

nrxs = ncomrx_thread.NcomRxThread()

nrxs.nrx['<ip>']['decoder'] will be an NcomRx class that can be used to
access the decoded data. For example:

  nrxs.nrx['192.168.2.62']['decoder'].nav['GpsTime']

Call nrxs.stop() to end, but note that the thread will be blocked on
data from the socket so it will only stop after data is received.
"""

import time
import socket
import ncomrx
import collections
import binascii
import threading


class NcomRxThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon_threads = True
        ncomrx.NcomRx.__init__(self)
        self.keepGoing = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 3000))
        self.nrx = {}
        self.moreCalcs = [] # List of calculation functions to expand nrx, see ncomrx
        self.start()
            
    def run(self):
        while(self.keepGoing):
            # Get data from socket
            nb, addrport = self.sock.recvfrom(256) # New bytes
            myTime = time.monotonic() # Grab time asap
            
            addr = addrport[0] # Just grab the IP address, not port
            
            # Is this a new IP address
            if addr not in self.nrx:
                # Then create a new crclist and decoder in nrx
                self.nrx[addr] = {
                    'crcList': collections.deque(maxlen=200),
                    'decoder': ncomrx.NcomRx()
                    }
                self.nrx[addr]['decoder'].moreCalcs = self.moreCalcs
                # Add IP address to connection, useful for user
                self.nrx[addr]['decoder'].connection['ip'] = addr
                self.nrx[addr]['decoder'].connection['repeatedUdp'] = 0
            
            # Under linux, UDP packets can be repeated, which messes up
            # the ncom decoding. Compute CRC and use it to identify
            # repeated packets
            crc = binascii.crc32(nb)
            if crc not in self.nrx[addr]['crcList']:
                self.nrx[addr]['crcList'].append(crc)                
                self.nrx[addr]['decoder'].decode(nb, machineTime=myTime)
                # And process all possible data
                while self.nrx[addr]['decoder'].decode(b'', machineTime=myTime):
                    pass
            else:
                self.nrx[addr]['decoder'].connection['repeatedUdp'] += 1
                                        
    def stop(self):
        self.keepGoing = False
