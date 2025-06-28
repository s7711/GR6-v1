# xnav.py
# Licensed under the MIT License – see LICENSE file for details.
"""
Functions and classes that deal with the xnav
- Receives data, decodes it, makes it available as dictionaries
- Automatically sends data to the web server
- Downloads the xNAV configuration
- Sends commands to the xNAV
"""

import threading
import ncomrx_thread
import time
import subprocess
import logging
import sys

class XNav:
    def __init__(self, ip_address):
        self.ip_address = ip_address
        self.nrxs = ncomrx_thread.NcomRxThread()    # Background ncom receiver and decoder
        self._sender_thread = threading.Thread(target=self.serve_nrxs, daemon=True)
        self._running = True
        self._sender_thread.start()
        self.ws = None  # Placeholder for web socket, to be set later
        threading.Thread(target=self.upload_xnav_config).start()

    def serve_nrxs(self):
        """
        serve_nrxs() loops forever serving ncom to the web socket.
        Run in a new thread
        """
        while self._running:
            time.sleep(0.5) # Sets the update rate for the messages
            
            if self.ws is not None:
                # nrxs.nrx is a dictionary of all the INSs found on the network
                # ... and the keys are the IP addresses
                # Form a list of the IP addresses
                devices = [ nrx for nrx in self.nrxs.nrx ] # list of keys/ip addresses
                self.ws.send("devices", devices) # Send to web server
                
                # For each INS extract the information from the decoder (NcomRx type)
                # Note that this implementation encodes everything in JSON, even if
                # it ends up that no websocket wants the data
                for addr, nrx in self.nrxs.nrx.items():      
                    self.ws.send("nav-data-"+addr, nrx['decoder'].nav) # Send to web server
                    self.ws.send("nav-status-"+addr, nrx['decoder'].status)
                    self.ws.send("nav-connection-"+addr, nrx['decoder'].connection)

    def upload_xnav_config(self):
        """ Upload the configuration from the xNAV and store it in
        folder static/xnav-config
        ".txt" is added as an extension so it can be viewed from
        the web browser
        """
        #TODO: This only works in Linux, not Windows
        if sys.platform == 'win32':
            return
        
        time.sleep(20.0) # Don't do this immediately
        
        fs = ["mobile.cfg", # main configuration file
            "mobile.gap", # GNSS antenna position
            "mobile.gpa", # GNSS antenna position accuracy
            "mobile.vat", # Vehicle attitude
            "mobile.vaa", # Vehicle attitude accuracy
            "mobile.att", # (GNSS) antenna attitude
            "mobile.ata", # (GNSS) antenna attitude accuracy
            # TODO: There are several other configuration files
            # that need to be added
            ]
        
        # Create/remove files in the directory
        cp = subprocess.run( ['mkdir', 'static/xnav-config'] )
        cp = subprocess.run( 'rm static/xnav-config/*', shell=True )
        
        for f in fs:
            cp = subprocess.run( ['curl', f'ftp://{self.ip_address}/{f}', '-o',
            f'static/xnav-config/{f}.txt'], capture_output=True )
            if cp.returncode == 0:
                logging.info(f"Uploaded config file {f}")
            else:
                logging.info(f"Cannot upload config file {f}")



