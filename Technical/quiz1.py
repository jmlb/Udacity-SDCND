#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 10 14:21:41 2016

@author: jmlbeaujour
"""

"""You can use this class to represent how classy someone
or something is.
"Classy" is interchangable with "fancy".
If you add fancy-looking items, you will increase
your "classiness".
Create a function in "Classy" that takes a string as
input and adds it to the "items" list.
Another method should calculate the "classiness"
value based on the items.
The following items have classiness points associated
with them:
"tophat" = 2
"bowtie" = 4
"monocle" = 5
Everything else has 0 points.
Use the test cases below to guide you!"""

class Classy(object):
    def __init__(self):
        self.items = []
        self.classy_dict = {"tophat": 2, "bowtie": 4, "monocle": 5}
        self.my_points = 0
    
    def addItem(self, single_item):
        self.items = self.items + [single_item]
        return self.items
        
    def getClassiness(self):
        self.my_points = 0
        if len(self.items) != 0:
            for this_item in self.items:
                if this_item in self.classy_dict.keys():
                    print("item: {} and points: {}".format(this_item, self.classy_dict[this_item]))
                    self.my_points += self.classy_dict[this_item]
            return self.my_points
        else:
            return None
# Test cases
me = Classy()

# Should be 0
print(me.getClassiness())

me.addItem("tophat")
# Should be 2
print(me.getClassiness())

me.addItem("bowtie")
me.addItem("jacket")
me.addItem("monocle")
# Should be 11
print(me.getClassiness())

me.addItem("bowtie")
# Should be 15
print(me.getClassiness())