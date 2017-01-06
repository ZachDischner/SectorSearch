# Drone Airspace Coding Challenge

# Given a square airspace, 128km x 128km, and N=10,000 drones occupying the airspace
# our challenge is to efficiently compute how many drones are flying too close to one 
# another. 

# Drone positions will be provided as an Nx2 array of [x,y] coordinates (in meters).
# Drones must maintain a horizontal separation of radius 0.5km from other drones. 

# If a drone is within 0.5km of another drone, both are "in conflict".
# Have count_conflicts return the total number of drones that are in a conflicted state. 
# Not the total number of conflicts).

# Do all of your work and testing in this pad. 
# Some common libraries can be imported, but not all, so relying on niche algorithm won't work. 
# This is very solvable with standard python libraries, several ways. 

# Coding style, readability, scalability, and documentation all matter! Consider the 
# computational complexity of your solution. 

# The N^2 answer can be coded up in 5 minutes and # 10 lines; we'd like to see something better!


"""Zach Dischner Sector Search Algorithm:

My method here will attempt to reduce the typical N^2 solution by breaking the entire search area into overlapping grids (called 'sectors' henceforth) to search in. 
AKA we know the drones in the top left of the area aren't close to the drones in the bottom right. So just search in local overlapping areas guranteed to capture any conflict radius drones. 

This will reduce the calculation complexity from N^2 to F*(N/F)^2 (assuming locations are more or less normally distributed), where F is the number of fields we're breaking the overall search area into, at the expense of some extra memory tracking which drones are in which field, etc. 

Visual:
     ____________________________________________________...AIRSPACE_SIZE
    |    *A      |            |*D          |             ...  
    |            |          *C|            |             ...
    |-------Sector 1----------|            |             ...
    |       *B   |            |     *E     |             ...
    |            |-------Sector 2----------|             ...
    |____________|____________|____________|_____________...
    |            |            |            |             ...
    |            |            |            |             ...
    |            |            |            |             ...
    ...  
    
    Drones in each sector:
        Sector 1 ==> [A,B,C]
        Sector 2 ==> [C,D,E]
    
    Once split up, we can examine each sector for drones in conflict. The key here is that the sectors overlap. Without it Sector 2 would start right where Sector 1 ended, and we would never know that drone *C and *D are actually in conflict since they are on the edges of each. 
    

Algorithm Qualities:
    + Simple: Doesn't require or warrant a published paper to explain
    + Portable-ish: Uses simple data structures, replicatable anywhere with any other language
    + Easily parallelizable! Once broken into sections, each section can be examined for conflicts independantly, and the results merged together. 
    + Recursable: What if a sector has too many drones in it to efficiently compute conflicts? Break that area into sectors! (not implemented that way here but easily architectable)
    + "Online": What if we get new readings for a few drones? Don't recompute the whole space, just update the sectors they are in (or move to) and recompute conflicts for those sectors
    - Requires data/memory overhead. Building dictionaries and sectors from scratch takes much longer than the actual conflict searching through each sector when dealing with not that many drones.
    - Certainly more complicated than the brute force N^2 approach
    - There are probably smarter searching algorithms out there (scipy.spacial search trees come to mind)
    - Full Overlap between sectors is overkill. Lots of room to optimize algo throughout

Testing:
    In basic testing here, Sector Search begins to overtake the brute force approach when searching through as few as 100 drones, and by a few thousand, it blows the brute force method away!
"""
import random
import numpy as np
from copy import copy
import time
random.seed(1)  # Setting random number generator seed for repeatability

NUM_DRONES = 10000
AIRSPACE_SIZE = 128000  # Meters.
CONFLICT_RADIUS = 500  # Meters.

###### Define some helper functions and classes
class Drone(dict):
    """Super simple, basically a dictionary with two fields and a conflict status attribute
    Could see this growing though so using a basic class for easier expansion.
    """
    def __repr__(self):
        return "Drone {} @ (x,y) {}".format(self['drone_id'],self['coords'])
    
    def __init__(self,coords,drone_id):
        """Initialize with a set of coordinates and an identifier"""
        self['coords'] = coords
        self['drone_id'] = drone_id    
            
def dist(p1,p2):
    """Calculate euclidian distance (norm) between two points"""
    return sum([(ax1-ax2)**2.0 for ax1,ax2 in zip(p1,p2)])**0.5
    # Same as numpy: return (sum((np.array(p1)-np.array(p2))**2))**0.5

###### Sector breaking functions
def split_into_sectors(airspace_size=AIRSPACE_SIZE, conflict_radius=CONFLICT_RADIUS, pad_mult=10):
    """Split square space into smaller grids.
    
    Each sector is a square space with sides `conflict_radius`*`padding`, with 50% overlap
    with surrounding sectors. 50% is overkill, ideally we could just overlap by the 
    `conflict_radius`, but safety factors are nice
    
    Kwargs:
        airspace_size:    (numeric) Length of one dimension of the field we're examining
        conflict_radius:  (numeric) Distance which defines a conflict. Same units as airspace_size
        pad_mult:         (padding) How many `conflict_radius`s wide to make the sectors
    
    Returns:
        boundaries:       (list) List of boundaries/indices of sector edges
        sectors:          (dict) Dictionary of sectors of form:  sectors[(x,y)] = []
    
    Example:
        split_into_sectors(airspace_size=20, conflict_radius=4, pad_mult=2) 
        
        Out:
        ([0, 8, 16, 20], 
         {(0, 0): [], 
          (0, 1): [], 
          (0, 2): [], 
          (0, 3): [], 
          (0, 4): [], 
          (1, 0): [], 
          (1, 1): [],...  
    """
    ###### Create Fields
    ## Get indices of grid as if overlayed on AIRSPACE_SIZE square space
    boundaries = range(airspace_size)[::conflict_radius*pad_mult]
    boundaries.append(airspace_size) # So we don't lose any stragglers
    
    ## Form dictionary to store associated drones in
    sectors = {(x,y):[] for x in range(len(boundaries)+1) for y in range(len(boundaries)+1)} 
    return boundaries, sectors
    
def map_coordinate(boundaries, position):
    """Determine the the sector number that `position` falls in within list of 
    `boundaries`, assuming that the `indices` feature overlapping field coordinates. 
    
    50% overlap between sectors is used to gurantee full coverage (though for this
    problem, we could probably get away with less, so long as the overlap size is 
    bigger than the overlap...). Thats just trickier though

    Visual:
    boundaries  [0    10      20    30      40...] 
                 |--sector 1--|
                       |--sector 2--|
                               |--sector 3--| ...

    So the 'position' 17 would fall in sector 1 AND sector 2. 

    Note that on the edges, this will say that the `position` is in a ficticious sector outside the
    bounds of the array. NBD, since this is just used to help group `boundaries`. Simplifies
    outside logic a bit so we'll keep it this way.
    """
    try:
        field = next(ix for ix, pivot in enumerate(boundaries) if pivot > position)
    except StopIteration:
        field = len(boundaries)-1
    
    return field-1,field

###### Main functions
def get_conflicts(drones, conflict_rad=CONFLICT_RADIUS, debug=False):
    """Return any Drones in conflict with each other in the list of Drones provided. 
    
    Simple construction, more careful thought and time would probably yield a prettier
    generator based solution, but this is pretty readable and my first 'smart' attempts weren't working out.
    This is basically the N^2 (actually N log(N)?) neighbors search through a list of `drones`
    
    Args:    
        drones:    list of Drone objects (just because we use the `conflict` attribute. Could change this to
                    be a dict with coords and drone_id keys and skip the pre-conflicting check)
    
    Kwargs:
        conflict_rad:    (numeric) Distance in same units as drone coordinate locations to define conflict
                                    threshold as
        debug:           (bool) Set to print out conflicts and pre-checked drones as they are processed
    
    Returns:
        conflicts: list of drone IDs in conflict (if any)
    """
    conflicts = []
    if len(drones) < 2:
        return conflicts
    
    ###### Compare drone against eachother to calculate distances
    # Only examine each combo once.
    for ax in range(len(drones)):
        for bx in range(ax+1,len(drones)):
            droneA = drones[ax]
            droneB = drones[bx]

            ## Calculate distance, flag both drones as conflicted ones 
            distance = dist(droneA['coords'],droneB['coords'])
            if distance < conflict_rad:
                if debug: print "Conflict! DroneA {} and DroneB {} are {}m apart! ".format(droneA['drone_id'], droneB['drone_id'], distance)
                conflicts.append(droneA['drone_id'])
                conflicts.append(droneB['drone_id'])

    ## Return unique list of conflicting drone ids. 
    #     Actually faster than numpy unique if the lists are small. 
    return [v for ix,v in enumerate(conflicts) if v not in conflicts[ix+1:]] # Changes the order of conflicts, but that is okay here
    # return list(np.unique(conflicts))

def count_conflicts(drones, conflict_radius,debug=False,limit=-1):
    """Count the number of drones that are in conflict over an airspace
    
    Logic:
        1. Convert raw coordinate pairs into a list of handier Drone() objects
        2. Split the big airfield into smaller overlapping sectors
        3. Associate each drone with sectors it resides in
        4. Examine each sector for drones within that are in conflict
    
    Args:
        drones:            (list of (x,y)) Coordinates of each drone. (meh namespace choice here just to stick with original provided naming convention)
        conflict_radius:   (numeric) Radius (in same units as drone coords) that defines a conflict

    Kwargs:
        debug:             (bool) Set to print out debug statistics throughout (not recomended for big searches)
        limit:             (int)  Limit on number of drones to process, here for debugging/benchmarking
    
    Returns:
        num_conflicts:     (int) Number of drones in conflict
    """
    assert -1 <= limit <= len(drones), "Limit to number of drones should not exceed total number of drones available"
    
    ## Create list of drone objects
    start_pre = time.time()
    drones = [Drone(drone,drone_id) for drone_id,drone in enumerate(drones)]
    
    ## Obtain field subsampling objects
    boundaries, sectors = split_into_sectors()
    
    ## Associate each drone with a sector
    for drone in drones[0:limit]:
        ## Assign drone to each overlapping sectors[(x,y)]
        xs = map_coordinate(boundaries, drone['coords'][0])    # returns tuple of assocaited X sectors (s1,s2)
        ys = map_coordinate(boundaries, drone['coords'][1])    # returns tuple of associated Y (s9,s10)
        
        # Assemble xs and ys into 2d coordinates that drones fall in: (s1,s9),(s1,s10),(s2,s9),(s2,s10)
        for sector_loc in [(x,y) for x in xs for y in ys]:
            sectors[sector_loc].append(drone)
        
    if debug: print "Preprocessing and forming all data structures took {:3.5f} seconds".format(time.time()-start_pre)
    
    ## Examine each sector for conflicts
    conflicts = []
    start_sector = time.time()
    for index, sector_drones in sectors.iteritems():
        conflicts += get_conflicts(sector_drones,conflict_rad=conflict_radius)
    
    num_conflicts = len(np.unique(conflicts))
    if debug: print "Sector Conflict Processing {} drones took {:3.5f} seconds. Number of conflicts: {}".format(limit,time.time()-start_sector, num_conflicts)
    
    ## Handy for benchmark testing, compare against raw, unsectored search through entire space
    if limit != -1:        
        start_batch = time.time()
        batch_conflicts = get_conflicts(drones[0:limit],conflict_rad=conflict_radius)
        if debug: print "\nBatch Processing all {} drones at once took {:3.5f} seconds. Number of conflicts: {}".format(limit, time.time()-start_batch,len(np.unique(batch_conflicts)))
    
    return num_conflicts

def gen_coord():
    return int(random.random() * AIRSPACE_SIZE)

positions = [[gen_coord(), gen_coord()] for i in range(NUM_DRONES)]
conflicts = count_conflicts(positions, CONFLICT_RADIUS)
print "Drones in conflict: {}".format(conflicts)

