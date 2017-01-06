# Implement the 3 functions belows so the code executes and produces the expected output.
# Do not include any other imports.
# Do not use any Python built-in functions like.
# Is guarenteed to execute to completion.

import time
import random

## YOUR CODE ONLY BELOW HERE
# 1. Implement this function
def enum(iterator):
    """Doesn't have to be an iterator, but why not?! Saves memory"""
    iternum = 0
    for value in iterator:
        yield iternum, value
        iternum += 1

# 2. Implement this function (a generator)
def stream_objects():
    while True:
        yield Object()
    
# 3. Implement this function
def timetaken(func):
    def time_me():
        start = time.time()
        ret = func()
        end = time.time()
        print "Run function took {:3.5f} seconds.".format(end-start)
        return ret
    # return modified function for execution
    return time_me
## YOUR CODE ONLY ABOVE HERE


## DO NOT MODIFY ANYTHING BELOW HERE
class Object():
    def __init__(self):
        self.complete = random.random() < 0.2

    def is_complete(self):
        return self.complete

@timetaken
def run():
    for index, current in enum(stream_objects()):
        if current.is_complete():
            return index

print 'Expected output:'
print "Run function took # seconds."
print 'Final object was at index #.'

print 'Actual output:'
final_index = run()
print 'Final object was at index {}'.format(final_index)