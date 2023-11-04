import math
class local_planning():
        # Scan bins 
    def __init__(self, distance, safe_distance, width_of_bin_0, max_speed, n_bins, gps_accurate):
        self.distance = distance
        self.safe_distance = safe_distance
        self.width_of_bin_0 = width_of_bin_0
        self.max_speed = max_speed
        self.n_bins = n_bins
        self.gps_accurate = gps_accurate

    def get_bins(self, lidar, n_bins, angle_of_b):
        bins = [] 
        safe_bins = []
        points_in_bin = [] 
        points_in_safe_bin = []
        for bin in range(int(n_bins)):
            bins.append(0)
            safe_bins.append(0)
            points_in_bin.append(0)
            points_in_safe_bin.append(0)
        vectors = lidar.getVectors()
        for vector in vectors:
            if vector[0] <= 90 or vector[0] >=270:
                if vector[0] >= 270:
                    angle_ = 360 - vector[0]
                else:
                    angle_ = vector[0]
                rad = math.radians(angle_)
                if vector[1]*math.sin(rad) <= self.width_of_bin_0/2 and vector[1]*math.cos(rad) <= self.distance:
                    points_in_bin[0] += 1
                if vector[1]*math.sin(rad) <= self.width_of_bin_0/2 and vector[1]*math.cos(rad) <= self.safe_distance:    
                    points_in_safe_bin[0] += 1
                    
            if vector[0] <= 360 - angle_of_b/2 and vector[0] >= angle_of_b/2:
                bin = int((angle_of_b/2+vector[0])/angle_of_b)
                if vector[1] <= self.distance: 
                    points_in_bin[bin] += 1
                if vector[1] <= self.safe_distance:
                    points_in_safe_bin[bin] += 1
                    
        for bin in range(int(n_bins)): 
            if points_in_bin[bin] >= 2:
                bins[bin] = 1
            if points_in_safe_bin[bin] >= 2:
                safe_bins[bin] = 1
                
        return bins, safe_bins
        
    def compute_desired_bins( beta, n_bins, angle_of_b, bins, safe_bins):
        bin_id = 0
        if beta < 0: #. beta in range (0,360) 
            beta += 360
        index = int((beta+angle_of_b/2) / angle_of_b)
        if index > n_bins-1:
            index = 0
        for i in range(n_bins//2):
            if(bins[(index+i)%n_bins] == 0):
                bin_id = (index+i)%n_bins
                break
            if(bins[index-i] == 0):
                bin_id = index-i
                if bin_id < 0:
                    bin_id = bin_id + n_bins 
                break
        if bins == [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]:
            return bin_id, False
        
        if bin_id < n_bins//2:
            for beta in range(0, bin_id - 1):
                if safe_bins[beta] == 1:
                    return bin_id, False
            return bin_id, True
        elif bin_id > n_bins//2:
            for beta in range(bin_id + 1, n_bins):
                if safe_bins[beta] == 1:
                    return bin_id, False
            return bin_id, True
        else:
            return bin_id, False

    def check_distance(self, lat_end, lon_end, lat_start, lon_start):
        lat_end = math.radians(lat_end)
        lon_end = math.radians(lon_end)
        lat_start = math.radians(lat_start)
        lon_start = math.radians(lon_start)
        
        d_lat = lat_end - lat_end
        d_lon = lon_end - lon_start
        angle = math.sin(d_lat / 2) ** 2 + math.cos(lat_end) * math.cos(lat_end) * math.sin(d_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(angle), math.sqrt(1 - angle))
        R = 6371000  # Approximate radius of the Earth in meters
        distance = R * c 
        print(distance)
        if distance < self.gps_accurate:
            return True
        return False 
    
    def go_place(self, Car, lidar, lat_end, lon_end, lat_start, lon_start):
        safety = 3
        lat_end = math.radians(lat_end)
        lon_end = math.radians(lon_end)
        lat_start = math.radians(lat_start)
        lon_start = math.radians(lon_start)
        angle_of_b = 360/self.n_bins
        d_lon = lon_end - lon_start
        
        # Calculate the bearing using the haversine formula
        y = math.sin(d_lon) * math.cos(lat_end)
        x = math.cos(lat_start) * math.sin(lat_end) - math.sin(lat_start) * math.cos(lat_end) * math.cos(d_lon)
        initial_bearing = math.atan2(y, x)

        # Convert the bearing from radians to degrees
        initial_bearing = math.degrees(initial_bearing)
        destination_angle = (initial_bearing + 360) % 360 
        current_angle = Car.getEuler('yaw') 
        
        # obstacle avoidance 
        bins, safe_bins = self.get_bins(lidar, self.n_bins, angle_of_b)
        beta = destination_angle - current_angle
        bin_id, success = self.compute_desired_bins( beta, self.n_bins, angle_of_b, bins, safe_bins)
        angle = bin_id * angle_of_b
        if angle > 180:
            angle = angle - 360
        if success:
            if abs(angle) <= 15:
                steering = 0
            elif 15 < abs(angle) and abs(angle) <= 40:
                steering = 0.35
            elif 40 < abs(angle) and abs(angle) <= 65:
                steering = 0.7
            elif 65 < abs(angle) and abs(angle) <= 90:
                steering = 1
            else:
                steering = 1
                            
            if angle <= 0:
                steering = -steering
            
            for bin in range(-self.n_bins//4 + 1, self.n_bins//4):
                if safe_bins[bin] == 1:
                    safety = 1
                    break
            if safety > 1:
                for bin in range(-self.n_bins//4 + 1, self.n_bins//4):
                    if bins[bin] == 1:
                        safety = 2
                        break
        else:
            steering = 0.0
            safety = 0.0  

        if safety == 3:
            speed = self.max_speed
        elif safety == 2:   
            speed = self.max_speed*4/5
        elif safety == 1:   
            speed = self.max_speed*3/5    
        else:
            speed = 0.0  
        print(f"bin_id: {bin_id}, turn: {angle}")
        print(f"steering: {steering}, safety: {safety}")    
        return steering, speed