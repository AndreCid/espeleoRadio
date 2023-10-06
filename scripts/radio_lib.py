class RadioLib:
    def __init__(self):

        # Starting variables

        # Class variables
        """
        RSSI range:
            Between 60 and 100 is a good signal
        dBm range:
            Between -20 and -70 is a good signal 
                OR between 80 and 30 (adding 100)
        """
        
        # Limits of radio readings according to unit
        self.radio_max_RSSI = 100
        self.limit_max_dBm = -15
        self.radio_max_dBm = self.limit_max_dBm + 100

        self.radio_min_RSSI = 60
        self.limit_low_dBm = -80
        self.radio_min_dBm = self.limit_low_dBm + 100


        
        # Limits of the marker array color
        self.green_max = 1
        self.green_min = 0

        # Ranges of the radio readings 
        self.OldRange_RSSI = 0
        self.OldRange_dBm = 0
        self.NewRange = 0

        self.get_range()

    
    """
    Function to get the limits of the radio range
    """
    def get_limits(self):
        if type:
            return [self.limit_low_dBm, self.limit_max_dBm]
        else:
            return [self.radio_min_RSSI, self.radio_max_RSSI]
        
    """
    Function to remap the radio signal value (from 0 to 100) to a value between  0 and 1
        If is dBm mode, first we convert the the negative value to positive
        0dbm = 100 and -100dbm = 0
    """
    
    def get_rgb_from_rssi(self, rssi, type):
        if type:
            rssi = rssi + 100
            if rssi < 0:
                rssi = 0
            rgb = (((rssi - self.radio_min_dBm) * self.NewRange) / self.OldRange_dBm) + self.green_min 
        else:
            rgb = (((rssi - self.radio_min_RSSI) * self.NewRange) / self.OldRange_RSSI) + self.green_min
        return rgb

    """
    Function to remap the rgb value (from 0 to 1) to a value between  0 and 100
        If is dBm mode, we recalculate to convert to negative again
        
    """
    def get_rssi_from_rgb(self, rgb, type):
        if type:
            intensity = (((rgb - self.green_min) * self.OldRange_dBm) / self.NewRange) + self.radio_min_dBm
            intensity = intensity - 100  
        else:
            intensity = (((rgb - self.green_min) * self.OldRange_RSSI) / self.NewRange) + self.radio_min_RSSI
        return intensity

    """
    Calculate the ranges for the remap rssi/rbg function
    """
    def get_range(self):
        self.OldRange_RSSI = self.radio_max_RSSI - self.radio_min_RSSI
        self.OldRange_dBm = self.radio_max_dBm - self.radio_min_dBm
        self.NewRange = self.green_max - self.green_min

    """
    Function to receive the x and y coordinates in meters, and transform in i and j index of the map matrix
    """
    @staticmethod
    def map_to_coordinates(x, y, map_origin, map_resolution):
        i = (x - map_origin[0]) / map_resolution
        j = (y - map_origin[1]) / map_resolution
        return [round(i), round(j)]

    """
    Function to receive the i and j index of the map matrix and transform in x and y coordinates in meters 
    """
    @staticmethod
    def coordinates_to_map(i, j, map_origin, map_resolution):
        x_map = map_origin[0] + (i * map_resolution)
        y_map = map_origin[1] + (j * map_resolution)
        return [x_map, y_map]

    """
    Function that receives i and j map matrix index and transform into a single array index 
    """
    @staticmethod
    def map_idx(sx, i, j):
        return (sx * j) + i
