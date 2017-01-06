## Sector Search
Question: Given 10,000 aircraft flying in a 128x128 km airspace, determine how many of them are flying to close to eachother

### Solution - Sector Search
See `SectorSearch.py` for discussion of implementation. This was a coding challenge I had fun with one morning that I had fun with, what a hoot!

## Run me 
Simple! Just run from the command line with a Python 2/3 interpreter. The only thing output is the number of aircraft in conflict. I should put in proper `argparse` kwargs to the call, to show a bit more of what is going on underneath. Including the fact that the O(N^2) simple search starts to become intolerably slower for many items to consider. 

```
>>> python SectorSearch.py
Drones in conflict: 3787
```