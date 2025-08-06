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
from config import CFG


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
        self.packets_sent = 0
        self.ws = None

    def user_command(self, message):
        """
        User command to update the GAD queue.
        This is called from the webpage to send commands to the GAD thread.
        """
        if message.startswith('#'):
            self.gad_queue.put(message)

    def serve_gad(self):
        """
        Sends oxts gad (generic aiding) messages
        Runs in a new thread
        Receives commands from queue
        """
        gh = oxts_sdk.GadHandler()
        gh.set_encoder_to_bin()    
        gadPkts = {}
        
        while self._running:
            time.sleep(0.5) # Sets the rate for updates

            # Process the gad queue
            # If there is anything in the queue then add it to gadPkts{}
            # (Or remove existing gadPkts entries using "stop")
            more = True
            while more:
                try:
                    command = self.gad_queue.get_nowait()
                except:
                    # Assume that the queue is now empty
                    more = False
            
                if more == True:
                    # Split up the command (uses spaces as a delimiter)
                    args = command.split()
                    
                    # Interpret the command
                    # In a try block to catch user errors, and anything else
                    try:
                        ### vel_ned
                        if args[0] == '#vel_ned':
                            if len(args) == 5 or len(args) == 4:
                                # #vel_ned vn ve vd [acc]
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
                                gadPkts["gv"] = gv
                            elif args[1] == 'stop':
                                del gadPkts["gv"]
                        
                        elif args[0] == '#vel_odom':
                            if len(args) == 5 or len(args) == 4:
                                # #vel_odom vf vr vd [acc]
                                gv = oxts_sdk.GadVelocity(132)
                                gv.vel_odom = [ float(v) for v in args[1:4] ]
                                if len(args) == 5:
                                    gv.vel_odom_var = [float(args[4]),float(args[4]),float(args[4]),0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                                else:
                                    # Default to a covariance of 0.1 ~ 31.6cm/s
                                    gv.vel_odom_var = [0.1,0.1,0.1,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                                gv.set_time_void()
                                gv.aiding_lever_arm_fixed = [0.0,0.0,0.0]
                                gv.aiding_lever_arm_var = [0.01,0.01,0.01]
                                gadPkts["gv"] = gv
                            elif args[1] == 'stop':
                                del gadPkts["gv"]
                        
                        ### pos_geo    
                        elif args[0] == '#pos_geo':
                            if len(args) == 4:
                                # #pos_geo lat lon alt
                                gp = oxts_sdk.GadPosition(129)
                                gp.pos_geodetic = [ float(v) for v in args[1:4] ]                    
                                gp.pos_geodetic_var = [0.1,0.1,1.0,0.0,0.0,0.0] # Note: using 3 terms does not appear to work
                                gp.set_time_void()
                                gp.aiding_lever_arm_fixed = [0.0,0.0,0.0]
                                gp.aiding_lever_arm_var = [0.001,0.001,0.001]
                                gadPkts["gp"] = gp
                            elif args[1] == 'stop':
                                del gadPkts["gp"]
                        
                        ### pos_att
                        elif args[0] == '#att':
                            if len(args) == 4:
                                ga = oxts_sdk.GadAttitude(131)
                                ga.att = [ float(v) for v in args[1:4] ]                    
                                ga.att_var = [10.0,100.0,100.0]
                                ga.set_time_void()
                                ga.set_aiding_alignment_optimising()
                                ga.aiding_alignment_var = [5.0,5.0,5.0]
                                gadPkts["ga"] = ga
                            elif args[1] == 'stop':
                                del gadPkts["ga"]

                        elif args[0] == '#att2':
                            if len(args) == 4:
                                ga = oxts_sdk.GadAttitude(131)
                                ga.att = [ float(v) for v in args[1:4] ]                    
                                ga.att_var = [10.0,10.0,10.0]
                                ga.set_time_void()
                                ga.aiding_alignment_fixed = [0.0, 0.0, 0.0]
                                ga.aiding_alignment_var = [5.0,5.0,5.0]
                                gadPkts["ga"] = ga
                            elif args[1] == 'stop':
                                del gadPkts["ga"]
                        
                    except Exception as e:
                        # Note this will show an error if "stop" is sent
                        # when there is no generic aiding message matching
                        # the stop request.
                        logging.warning(f"Command error: {e}")            
            
            # Each gad packet that needs to be sent
            for k,pkt in gadPkts.items():
                try:
                    gh.set_output_mode_to_udp(CFG['InsIp'])
                    gh.send_packet(pkt)
                    self.packets_sent += 1
                except Exception as e:
                    logging.warning(e)
                    del gadPkts[k]                        # Don't bother with this again

            if self.ws is not None:
                self.ws.send("gad_fake", {'GadFakePacketsSent': self.packets_sent,
                                          'GadFakePacketsListLen': len(gadPkts)})
    
    def stop(self):
        """
        Stop the GAD thread.
        This will stop sending updates and terminate the thread.
        """
        self._running = False
        self._thread.join()  # Wait for the thread to finish

