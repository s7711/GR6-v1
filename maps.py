# maps.py
# Licensed under the MIT License – see LICENSE file for details.
"""
Maps class:
* mp = Maps() to initialise the maps
* mp.directory - sets the directory where the maps are loaded/saved
* mp.resolution - sets the resolution (all maps share the same resolution)
* mp.origin_lat - sets the south-east corner for each map
* mp.origin_lon - south/east corner
* mp.origin_alt - soath/east corner
* mp.maps - dictionary of maps
* mp.maps['ground_rgb'] returns a dictonary where the standard fields are:
    * mp.maps[xxx].data - numpy array representing the map
    * mp.maps[xxx].confidence - numpy array (floats) representing confidence from 0 to 1
"""
import os
import pickle
import logging
import numpy as np
from config import CFG
import ncomrx

class Maps:
    def __init__(self, map_list=None, directory="static/maps"):
        """
        Initialise the map class and load the maps.
        The map_list can be used to check that the right maps have been loaded
        and create a new map isn't loaded. map_list should be in the format:
        {map_name: (creator, {channels:<n>, fill:<value>, dtype:<dtype>})}
        creator can be create_map(), create_ground_rgb() value is passed to
        the creator function. See each function for the key/values it needs
        """
        self.directory = directory
        self.MapSizeMeters = CFG['MapSizeMeters']
        self.resolution = CFG['MapResolution']
        self.base_lat = CFG['MapBaseLLA'][0]
        self.base_lon = CFG['MapBaseLLA'][1]
        self.base_alt = CFG['MapBaseLLA'][2]
        self.map_size = int(self.MapSizeMeters / self.resolution)
        self.maps = {}  # Key: map name, Value: dict with fields 'data', 'confidence', etc.

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        
        self.load_all()

        if map_list is not None:
            for map_name, (creator_name, params) in map_list.items():
                if map_name not in self.maps:
                    creator_func = getattr(self, creator_name)
                    creator_func(map_name, params)

    def create_map(self, name, properties):
        """Creates a new blank map with the given shape and fill value"""
        if name in self.maps:
            logging.info(f"Map '{name}' already exists. Overwriting.")
        channels = properties.get("channels", 1)
        shape = (self.map_size, self.map_size) if channels == 1 else (self.map_size, self.map_size, channels)
        self.maps[name] = {
            "data": np.full(shape, properties['fill'], dtype=properties['dtype']),
            "confidence": np.zeros(shape[:2], dtype=float),
        }
        logging.info(f"[Maps]: Created new map {name}")

    def save_map(self, name):
        """Save a single map to disk using pickle"""
        if name not in self.maps:
            logging.info(f"Map '{name}' not found.")
            return
        filepath = os.path.join(self.directory, f"{name}.pkl")
        with open(filepath, "wb") as f:
            pickle.dump(self.maps[name], f)
            logging.info(f"[Maps]: Saved map {name}")

    def load_map(self, name):
        filepath = os.path.join(self.directory, f"{name}.pkl")
        if os.path.exists(filepath):
            with open(filepath, "rb") as f:
                loaded = pickle.load(f)

            self.maps[name] = loaded
            logging.info(f"[Maps]: Loaded map {name}")

        else:
            logging.info(f"Map '{name}' not found.")


    def save_all(self):
        """Save all loaded maps"""
        for name in self.maps:
            self.save_map(name)

    def load_all(self):
        """Load all .pkl maps in directory"""
        for filename in os.listdir(self.directory):
            if filename.endswith(".pkl"):
                name = filename[:-4]
                self.load_map(name)

    ###############################################################################
    # Special map creation functions
    def create_ground_rgb(self, name, properties):
        """
        Initializes a white RGB map with light grey gridlines every grid_spacing_m.
        Confidence is zero everywhere.
        """
        map_size = int(CFG["MapSizeMeters"] / CFG["MapResolution"])
        pixels_per_grid = int(properties['grid_spacing_m'] / CFG["MapResolution"])

        # White background
        rgb = np.full((map_size, map_size, 3), 255, dtype=np.uint8)
        grey = 200

        # Add grey gridlines
        for i in range(0, map_size, pixels_per_grid):
            rgb[i, :, :] = grey  # horizontal lines (northing)
            rgb[:, i, :] = grey  # vertical lines (easting)

        confidence = np.zeros((map_size, map_size), dtype=float)

        self.maps[name] = {
            "data": rgb,
            "confidence": confidence,
        }
        logging.info(f"[Maps]: Created new map {name}")

# Additional calculations for ncomrx
# Called (from ncomrx.py) each time an NCOM packet is received
# Add in MapLocal
def calcMapLocal( nrx ):
    MapLocal = ncomrx.LLA2NED(nrx.nav['Lat'] * ncomrx.RAD2DEG,
        nrx.nav['Lon'] * ncomrx.RAD2DEG, nrx.nav['Alt'],
        CFG['MapBaseLLA'][0], CFG['MapBaseLLA'][1], CFG['MapBaseLLA'][2])

    # Using an ENU co-ordinate, referenced as XYZ for mapping
    nrx.nav['MapLocalX'] = MapLocal['LocalE']
    nrx.nav['MapLocalY'] = MapLocal['LocalN']
    nrx.nav['MapLocalZ'] = MapLocal['LocalZ']
    nrx.nav['MapLocalHeading'] = nrx.nav['Heading']
