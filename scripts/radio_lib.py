class RadioLib:
    def __init__(self):

        # Starting variables

        # Class variables
        self.radio_max = 100
        self.radio_min = 82
        self.green_max = 1
        self.green_min = 0
        self.OldRange = 0
        self.NewRange = 0
        self.get_range()

    """
    Function to remap the radio signal value (from 0 to 100) to a value between  0 and 1
    """
    # def get_rgb_from_rssi(self, rssi, radio_min=None):
    def get_rgb_from_rssi(self, rssi):
        # self.radio_min = radio_min if radio_min is not None else self.radio_min
        rgb = (((rssi - self.radio_min) * self.NewRange) / self.OldRange) + self.green_min
        return rgb

    """
    Function to remap the rgb value (from 0 to 1) to a value between  0 and 100
    """
    def get_rssi_from_rgb(self, rgb):
        intensity = (((rgb - self.green_min) * self.OldRange) / self.NewRange) + self.radio_min
        return intensity

    """
    Calculate the ranges for the remap rssi/rbg function
    """
    def get_range(self):
        self.OldRange = self.radio_max - self.radio_min
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
