# gad_fake.py
# Licensed under the MIT License – see LICENSE file for details.
"""
THIS IS UNTESTED CODE

Used to send fake gad updates to the OXTS navigation system.
Can be used to test GAD, to keep the xNAV650 stable if GNSS is not available, etc.

Usage:
    gf = gad_fake.GadFake(CFG))
    gf.userCommand(message,path) - Updates from the webpage
    gf.stop() terminates the thread and stops sending updates.
"""
import queue
import time
import logging
import oxts_sdk
import threading


class GadFake:
    """
    GadFake class to send fake GAD updates to the OXTS navigation system.
    This is used to keep the xNAV650 stable if GNSS is not available, etc.
    """
    def __init__(self, CFG):
        self.gad_queue = queue.Queue()
        self.CFG = CFG
        self._thread = threading.Thread(target=self.serve_gad, daemon=True)
        self._running = True
        self._thread.start()

    def userCommand(self, ip, message):
        """
        User command to update the GAD queue.
        This is called from the webpage to send commands to the GAD thread.
        """
        # message is a string, ip is a string of the address for the GAD update
        # The message should be in the format "#command arg1 arg2 ..."
        # e.g. "#vel_ned 1.0 2.0 3.0" or "#pos_geo 51.5074 -0.1278 10.0"
        # or "#att 0.1 0.2 0.3"
        if message.startswith('#'):
            self.gad_queue.put((ip, message))

    # Serve_gad is used for "testing" and is not part of the normal aruco
    # marker generic aiding update. It can be used to send a regular fixed
    # position, velocity of attitude generic aiding update. This can be
    # useful while debugging or during initial testing if the camera or
    # markers are not working. The velocity update can be useful (as a
    # zero velocity update) to keep the INS roughly stationary while
    # markers are not visible.
    #
    # The loop runs on a timer (see time.sleep(0.5) below). A queue is used
    # to send new updates to the thread running serve_gad().
    def serve_gad(self):
        """
        Sends oxts gad (generic aiding) messages
        Runs in a new thread
        Receives commands from queue
        """
        gh = oxts_sdk.GadHandler()
        gh.set_encoder_to_bin()    

        # gadPkts is a dictionary of {'type_ip':type.ip address, 'pkt':gad_packet}
        # These are the packets that should be sent out on a regular basis
        # The type_ip field is used so the entry can be found, replaced and remove easily
        # and is formatted as "gp:192.168.2.62", where "gp" is:
        #   gp: gadPosition
        #   gv: gadVelocity
        #   etc.
        # Use split(":") to separate type and ip address
        gadPkts = {}
        
        while self._running:
            time.sleep(0.5) # Sets the rate for updates

            # Process the gad queue
            # If there is anything in the queue then add it to gadPkts{}
            # (Or remove existing gadPkts entries using "stop")
            more = True
            while more:
                try:
                    ip, command = self.gad_queue.get_nowait()
                except:
                    # Assume that the queue is now empty
                    more = False
            
                if more == True:
                    # Split up the command (uses spaces as a delimiter)
                    args = command.split()
                    
                    print("Update")

                    # Interpret the command
                    # In a try block to catch user errors, and anything else
                    try:
                        ### vel_ned
                        if args[0] == '#vel_ned':
                            if len(args) == 5 or len(args) == 4:
                                # #vel_ned vn ve vu [acc]
                                gv = oxts_sdk.GadVelocity(132)
                                gv.vel_ned = [ float(v) for v in args[1:4] ]
                                if len(args) == 5:
                                    gv.vel_ned_var = [float(args[4]),float(args[4]),float(args[4]),0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                                else:
                                    # Default to a covariance of 0.1 ~ 31.6cm/s
                                    gv.vel_ned_var = [0.1,0.1,0.1,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                                gv.set_time_void()
                                gv.aiding_lever_arm_fixed = [0.0,0.0,0.0]
                                gv.aiding_lever_arm_var = [0.01,0.01,0.01]
                                gadPkts["gv:"+ip] = gv
                            elif args[1] == 'stop':
                                del gadPkts["gv:"+ip]
                                
                        ### pos_geo    
                        elif args[0] == '#pos_geo':
                            if len(args) == 4:
                                # #pos_geo lat lon alt
                                gp = oxts_sdk.GadPosition(129)
                                gp.pos_geodetic = [ float(v) for v in args[1:4] ]                    
                                gp.pos_geodetic_var = [0.01,0.01,10.0,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                                gp.set_time_void()
                                gp.aiding_lever_arm_fixed = [0.0,0.0,0.0]
                                gp.aiding_lever_arm_var = [0.001,0.001,0.001]
                                gadPkts["gp:"+ip] = gp
                            elif args[1] == 'stop':
                                del gadPkts["gp:"+ip]
                        
                        ### pos_att
                        elif args[0] == '#att':
                            if len(args) == 4:
                                ga = oxts_sdk.GadAttitude(131)
                                ga.att = [ float(v) for v in args[1:4] ]                    
                                ga.att_var = [10.0,100.0,100.0]
                                ga.set_time_void()
                                ga.set_aiding_alignment_optimising()
                                ga.aiding_alignment_var = [5.0,5.0,5.0]
                                gadPkts["ga:"+ip] = ga
                            elif args[1] == 'stop':
                                del gadPkts["ga:"+ip]
                        
                    except Exception as e:
                        # Note this will show an error if "stop" is sent
                        # when there is no generic aiding message matching
                        # the stop request. The exception will be a key
                        # error
                        logging.warning(f"Command error: {e}")            
            
            # Each gad packet that needs to be sent to each ip address
            # in gadPkts with keys 'type_ip' and 'pkt'
            # Set the IP address and then send the packet
            for k,pkt in gadPkts.items():
                try:
                    type_ip = k.split(':')
                    gh.set_output_mode_to_udp(type_ip[1]) # Should be IP address
                    gh.send_packet(pkt)
                except Exception as e:
                    logging.warning(e)
                    del gadPkts[k]                        # Don't bother with this again
    
    def stop(self):
        """
        Stop the GAD thread.
        This will stop sending updates and terminate the thread.
        """
        self._running
        self._thread.join()  # Wait for the thread to finish
        self._running = False

