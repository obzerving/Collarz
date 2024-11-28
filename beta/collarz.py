#!/usr/bin/env python
# coding=utf-8
#
# Copyright (C) [2021] [Joseph Zakar], [observing@gmail.com]
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
"""
Given a set of parameters for two polygons, this program generates paper
models of (1) the two polygons; (2) a collar (divided into segments if desired)
represented by a strip with tabs and score lines; and (3) wrapper(s) for
covering the tabbed strip(s).
"""

import inkex
import math
import copy

from inkex import PathElement, Style
from inkex.paths import Move, Line, ZoneClose, Path
from inkex.elements._groups import Group

class pathStruct(object):
    def __init__(self):
        self.id="path0000"
        self.path=Path()
        self.enclosed=False
        self.style = None
    def __str__(self):
        return self.path
    
class pnPoint(object):
   # This class came from https://github.com/JoJocoder/PNPOLY
    def __init__(self,p):
        self.p=p
    def __str__(self):
        return self.p
    def InPolygon(self,polygon,BoundCheck=False):
        inside=False
        if BoundCheck:
            minX=polygon[0][0]
            maxX=polygon[0][0]
            minY=polygon[0][1]
            maxY=polygon[0][1]
            for p in polygon:
                minX=min(p[0],minX)
                maxX=max(p[0],maxX)
                minY=min(p[1],minY)
                maxY=max(p[1],maxY)
            if self.p[0]<minX or self.p[0]>maxX or self.p[1]<minY or self.p[1]>maxY:
                return False
        j=len(polygon)-1
        for i in range(len(polygon)):
            if ((polygon[i][1]>self.p[1])!=(polygon[j][1]>self.p[1]) and (self.p[0]<(polygon[j][0]-polygon[i][0])*(self.p[1]-polygon[i][1])/( polygon[j][1] - polygon[i][1] ) + polygon[i][0])):
                    inside =not inside
            j=i
        return inside

class Collar(inkex.EffectExtension):
    
    def add_arguments(self, pars):
        pars.add_argument("--usermenu")
        pars.add_argument("--unit", default="in",\
            help="Dimensional units")
        pars.add_argument("--polysides", type=int, default=6,\
            help="Number of Polygon Sides")
        pars.add_argument("--poly1size", type=float, default=5.0,\
            help="Size of Polygon 1 in dimensional units")
        pars.add_argument("--poly2size", type=float, default=3.0,\
            help="Size of Polygon 2 in dimensional units")
        pars.add_argument("--collarheight", type=float, default=2.0,\
            help="Height of collar in dimensional units")
            
        pars.add_argument("--halfpoly", type=inkex.Boolean, default=0,\
            help="Make only half the poly shape (if even number of sides)")
            
        pars.add_argument("--collarparts", type=int, default=1,\
            help="Number of parts to divide collar into")
        pars.add_argument("--dashlength", type=float, default=0.1,\
            help="Length of dashline in dimensional units (zero for solid line)")
        pars.add_argument("--tabangle", type=float, default=45.0,\
            help="Angle of tab edges in degrees")
        pars.add_argument("--linesonwrapper", type=inkex.Boolean, dest="linesonwrapper",\
            help="Put dashlines on wrappers")
        pars.add_argument("--circumscribed", type=inkex.Boolean, dest="circumscribed",\
            help="Use circumscribed diameter")
        pars.add_argument("--tabheight", type=float, default=0.4,\
            help="Height of tab in dimensional units")

    #draw SVG line segment(s) between the given (raw) points
    def drawline(self, dstr, name, parent, sstr=None):
        line_style   = {'stroke':'#000000','stroke-width':'0.25','fill':'#eeeeee'}
        if sstr == None:
            stylestr = str(Style(line_style))
        else:
            stylestr = sstr
        el = parent.add(PathElement())
        el.path = dstr
        el.style = stylestr
        el.label = name
        
    def makepoly(self, toplength, numpoly,polylimit):
      r = toplength/(2*math.sin(math.pi/numpoly))
      pstr = Path()
      for ppoint in range(0,polylimit):
         xn = r*math.cos(2*math.pi*ppoint/numpoly)
         yn = r*math.sin(2*math.pi*ppoint/numpoly)
         if ppoint == 0:
            pstr.extend([Move(xn,yn)])
         else:
            pstr.extend([Line(xn,yn)])
      pstr.extend([ZoneClose()])
      return pstr

    # Thanks to Gabriel Eng for his python implementation of https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    def findIntersection(self, x1,y1,x2,y2,x3,y3,x4,y4):
        px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) ) 
        py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )
        return px, py

    def insidePath(self, path, p):
        point = pnPoint((p.x, p.y))
        pverts = []
        for pnum in path:
            if pnum.letter == 'Z':
                pverts.append((path[0].x, path[0].y))
            else:
                pverts.append((pnum.x, pnum.y))
        isInside = point.InPolygon(pverts, True)
        return isInside # True if point p is inside path

    def makescore(self, pt1, pt2, dashlength):
        # Draws a dashed line of dashlength between two points
        # Dash = dashlength space followed by dashlength mark
        # if dashlength is zero, we want a solid line
        # Returns dashed line as a Path object
        apt1 = Line(0.0,0.0)
        apt2 = Line(0.0,0.0)
        ddash = Path()
        if math.isclose(dashlength, 0.0):
            #inkex.utils.debug("Draw solid dashline")
            ddash.extend([Move(pt1.x,pt1.y)])
            ddash.extend([Line(pt2.x,pt2.y)])
        else:
            if math.isclose(pt1.y, pt2.y):
                #inkex.utils.debug("Draw horizontal dashline")
                if pt1.x < pt2.x:
                    xcushion = pt2.x - dashlength
                    xpt = pt1.x
                    ypt = pt1.y
                else:
                    xcushion = pt1.x - dashlength
                    xpt = pt2.x
                    ypt = pt2.y
                done = False
                while not(done):
                    if (xpt + dashlength*2) <= xcushion:
                        xpt = xpt + dashlength
                        ddash.extend([Move(xpt,ypt)])
                        xpt = xpt + dashlength
                        ddash.extend([Line(xpt,ypt)])
                    else:
                        done = True
            elif math.isclose(pt1.x, pt2.x):
                #inkex.utils.debug("Draw vertical dashline")
                if pt1.y < pt2.y:
                    ycushion = pt2.y - dashlength
                    xpt = pt1.x
                    ypt = pt1.y
                else:
                    ycushion = pt1.y - dashlength
                    xpt = pt2.x
                    ypt = pt2.y
                done = False
                while not(done):
                    if(ypt + dashlength*2) <= ycushion:
                        ypt = ypt + dashlength         
                        ddash.extend([Move(xpt,ypt)])
                        ypt = ypt + dashlength
                        ddash.extend([Line(xpt,ypt)])
                    else:
                        done = True
            else:
                #inkex.utils.debug("Draw sloping dashline")
                if pt1.y > pt2.y:
                    apt1 = Line(pt1.x,pt1.y)
                    #apt1.x = pt1.x
                    #apt1.y = pt1.y
                    apt2 = Line(pt2.x,pt2.y)
                    #apt2.x = pt2.x
                    #apt2.y = pt2.y
                else:
                    apt1 = Line(pt2.x,pt2.y)
                    #apt1.x = pt2.x
                    #apt1.y = pt2.y
                    apt2 = Line(pt1.x,pt1.y)
                    #apt2.x = pt1.x
                    #apt2.y = pt1.y
                m = (apt1.y-apt2.y)/(apt1.x-apt2.x)
                theta = math.atan(m)
                msign = (m>0) - (m<0)
                ycushion = apt2.y + dashlength*math.sin(theta)
                xcushion = apt2.x + msign*dashlength*math.cos(theta)
                xpt = apt1.x
                ypt = apt1.y
                done = False
                while not(done):
                    nypt = ypt - dashlength*2*math.sin(theta)
                    nxpt = xpt - msign*dashlength*2*math.cos(theta)
                    if (nypt >= ycushion) and (((m<0) and (nxpt <= xcushion)) or ((m>0) and (nxpt >= xcushion))):
                        # move to end of space / beginning of mark
                        xpt = xpt - msign*dashlength*math.cos(theta)
                        ypt = ypt - msign*dashlength*math.sin(theta)
                        ddash.extend([Move(xpt,ypt)])
                        # draw the mark
                        xpt = xpt - msign*dashlength*math.cos(theta)
                        ypt = ypt - msign*dashlength*math.sin(theta)
                        ddash.extend([Line(xpt,ypt)])
                    else:
                        done = True
        return ddash

    def detectIntersect(self, x1, y1, x2, y2, x3, y3, x4, y4):
        td = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
        if td == 0:
            # These line segments are parallel
            return False
        t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/td
        if (0.0 <= t) and (t <= 1.0):
            return True
        else:
            return False

    def orientTab(self,pt1,pt2,height,angle,theta,orient):
        tpt1 = Line(pt1.x + orient[0]*height + orient[1]*height/math.tan(math.radians(angle)),pt1.y + orient[4]*height + orient[5]*height/math.tan(math.radians(angle)))
        tpt2 = Line(pt2.x + orient[2]*height + orient[3]*height/math.tan(math.radians(angle)),pt2.y + orient[6]*height + orient[7]*height/math.tan(math.radians(angle)))
        if not math.isclose(theta, 0.0):
            t11 = Path([Move(pt1.x,pt1.y),Line(tpt1.x, tpt1.y)])
            t12 = Path([Move(pt1.x,pt1.y),Line(tpt2.x, tpt2.y)])
            thetal1 = t11.rotate(theta, [pt1.x,pt1.y])
            thetal2 = t12.rotate(theta, [pt2.x,pt2.y])
            tpt1 = Line(thetal1[1].x,thetal1[1].y)
            #tpt1.x = thetal1[1].x
            #tpt1.y = thetal1[1].y
            tpt2 = Line(thetal2[1].x,thetal2[1].y)
            #tpt2.x = thetal2[1].x
            #tpt2.y = thetal2[1].y
        return tpt1,tpt2

    def makeTab(self, tpath, pt1, pt2, tabht, taba):
        # tpath - the pathstructure containing pt1 and pt2
        # pt1, pt2 - the two points where the tab will be inserted
        # tabht - the height of the tab
        # taba - the angle of the tab sides
        # returns the two tab points (Line objects) in order of closest to pt1
        tpt1 = Line(0.0,0.0)
        tpt2 = Line(0.0,0.0)
        currTabHt = tabht
        currTabAngle = taba
        testAngle = 1.0
        testHt = currTabHt * 0.001
        adjustTab = 0
        tabDone = False
        while not tabDone:
            # Let's find out the orientation of the tab
            if math.isclose(pt1.x, pt2.x):
                # It's vertical. Let's try the right side
                if pt1.y < pt2.y:
                    pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,0.0,[1,0,1,0,0,1,0,-1])
                    if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                       (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[-1,0,-1,0,0,1,0,-1]) # Guessed wrong
                    else:
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[1,0,1,0,0,1,0,-1]) # Guessed right
                else: # pt2.y < pt1.y
                    pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,0.0,[1,0,1,0,0,-1,0,1])
                    if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                       (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[-1,0,-1,0,0,-1,0,1]) # Guessed wrong
                    else:
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[1,0,1,0,0,-1,0,1]) # Guessed right
            elif math.isclose(pt1.y, pt2.y):
                # It's horizontal. Let's try the top
                if pt1.x < pt2.x:
                    pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,0.0,[0,1,0,-1,-1,0,-1,0])
                    if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                       (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[0,1,0,-1,1,0,1,0]) # Guessed wrong
                    else:
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[0,1,0,-1,-1,0,-1,0]) # Guessed right
                else: # pt2.x < pt1.x
                    pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,0.0,[0,-1,0,1,-1,0,-1,0])
                    if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                       (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[0,-1,0,1,1,0,1,0]) # Guessed wrong
                    else:
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[0,-1,0,1,-1,0,-1,0]) # Guessed right

            else: # the orientation is neither horizontal nor vertical
                # Let's get the slope of the line between the points
                # Because Inkscape's origin is in the upper-left corner,
                # a positive slope (/) will yield a negative value
                slope = (pt2.y - pt1.y)/(pt2.x - pt1.x)
                # Let's get the angle to the horizontal
                theta = math.degrees(math.atan(slope))
                # Let's construct a horizontal tab
                seglength = math.sqrt((pt1.x-pt2.x)**2 +(pt1.y-pt2.y)**2)
                if slope < 0.0:
                    if pt1.x < pt2.x:
                        pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,theta,[0,1,0,-1,-1,0,-1,0])
                        if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                           (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,1,0,-1,1,0,1,0]) # Guessed wrong
                        else:
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,1,0,-1,-1,0,-1,0]) # Guessed right
                    else: # pt1.x > pt2.x
                        pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,theta,[0,-1,0,1,-1,0,-1,0])
                        if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                           (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,-1,0,1,1,0,1,0]) # Guessed wrong
                        else:
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,-1,0,1,-1,0,-1,0]) # Guessed right
                else: # slope > 0.0
                    if pt1.x < pt2.x:
                        pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,theta,[0,1,0,-1,-1,0,-1,0])
                        if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                           (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,1,0,-1,1,0,1,0]) # Guessed wrong
                        else:
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,1,0,-1,-1,0,-1,0]) # Guessed right
                    else: # pt1.x > pt2.x
                        pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,theta,[0,-1,0,+1,-1,0,-1,0])
                        if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                           (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,-1,0,1,1,0,1,0]) # Guessed wrong
                        else:
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,-1,0,1,-1,0,-1,0]) # Guessed right
            # Check to see if any tabs intersect each other
            if self.detectIntersect(pt1.x, pt1.y, tpt1.x, tpt1.y, pt2.x, pt2.y, tpt2.x, tpt2.y):
                # Found an intersection.
                if adjustTab == 0:
                    # Try increasing the tab angle in one-degree increments
                    currTabAngle = currTabAngle + 1.0
                    if currTabAngle > 88.0: # We're not increasing the tab angle above 89 degrees
                        adjustTab = 1
                        currTabAngle = taba
                if adjustTab == 1:
                    # So, try reducing the tab height in 20% increments instead
                    currTabHt = currTabHt - tabht*0.2 # Could this lead to a zero tab_height?
                    if currTabHt <= 0.0:
                        # Give up
                        currTabHt = tabht
                        adjustTab = 2
                if adjustTab == 2:
                    tabDone = True # Just show the failure
            else:
                tabDone = True
            
        return tpt1,tpt2
        
        
    def makeback(self, toplength,botlength,numpoly,polylimit,collarht,tab_height,tab_angle,dashlength):
    #SUE CHANGE -- This function makes the piece for the back for when it is an incomplete polygon
      r = toplength/(2*math.sin(math.pi/numpoly))
      r2 = botlength/(2*math.sin(math.pi/numpoly))

      layer = self.svg.get_current_layer()
      pdstr = pathStruct()
      
      pdstr2 = Path()
      spaths = ""

      
      for ppoint in range(0,polylimit):
         xn = r*math.cos(2*math.pi*ppoint/numpoly)  #bigger poly
         yn = r*math.sin(2*math.pi*ppoint/numpoly)
         xnb = r2*math.cos(2*math.pi*ppoint/numpoly) #smaller poly
         ynb= r2*math.sin(2*math.pi*ppoint/numpoly)

         if ppoint == 0:
            #just save the first calculated points
            dxt1 = xn #larger x
            dyt1 = yn #larger y 
            dxb1 = xnb #smaller x
            dyb1= ynb  #smaller y

      # save the last points
      dxt2 = xn #these will keep getting replaced with the last calculated points
      dyt2 = yn
      dxb2 = xnb
      dyb2 = ynb
      #calculate the lengths between the first and last constructed nodes for top and bottom            
      diaglent = math.sqrt( (max(dxt2,dxt1)- min(dxt2,dxt1))**2 + (max(dyt2,dyt1)- min(dyt2,dyt1))**2) #length of larger diagonal
      diaglenb = math.sqrt( (max(dxb2,dxb1)- min(dxb2,dxb1))**2 + (max(dyb2,dyb1)- min(dyb2,dyb1))**2) #length of smaller diagonal
      dhalfleft = (diaglent - diaglenb)/2  #half the difference of the two diagonals
      dhalfright = diaglent-dhalfleft

      #Now build a string with the points  
      pdstr.path.extend([Move(0,0)])
      
      pdstr.path.extend([Line(diaglent,0)])
      
      pdstr.path.extend([Line(dhalfright,collarht)])
      
      pdstr.path.extend([Line(dhalfleft,collarht)])  
      
      pdstr.path.extend([ZoneClose()])
      
      pdstr2.extend(pdstr.path[0])

      #SUE added to make a wrapper for back
      self.drawline(str(pdstr.path),'backw',layer,sstr="fill:#ffdddd;stroke:#000000;stroke-width:0.25") #output the back piece  
      #####
      
      point1,point2 = self.makeTab(pdstr,pdstr.path[0],pdstr.path[1],tab_height,tab_angle)
      pdstr2.extend([point1])
      pdstr2.extend([point2])
      pdstr2.extend(pdstr.path[1])
      pdstr2.extend(pdstr.path[2])
      
      point1,point2 = self.makeTab(pdstr,pdstr.path[2],pdstr.path[3],tab_height,tab_angle)
      pdstr2.extend([point1])
      pdstr2.extend([point2])
      pdstr2.extend(pdstr.path[3])
      
      point1,point2 = self.makeTab(pdstr,pdstr.path[3],pdstr.path[0],tab_height,tab_angle)
      pdstr2.extend([point1])
      pdstr2.extend([point2])
      #pdstr2.append(pdstr.path[0])
      
      pdstr2.extend([ZoneClose()])
      
     
      scorepath = Path()

      spaths = self.makescore(pdstr.path[0],pdstr.path[1],dashlength)
      
      scorepath.extend(spaths)
      spaths = self.makescore(pdstr.path[2],pdstr.path[3],dashlength)
      scorepath.extend(spaths)
      spaths = self.makescore(pdstr.path[3],pdstr.path[0],dashlength)
      scorepath.extend(spaths)
      #put solid score lines into group with back piece
      if math.isclose(dashlength, 0.0):
        bgroup = Group()
        bgroup.label = 'group'+'back'
        self.drawline(str(pdstr2),'back',   bgroup,sstr="fill:#eeeeee;stroke:#000000;stroke-width:0.25") # Output the shape

        self.drawline(str(scorepath), 'bkscore',bgroup,sstr="fill:None;stroke:#000000;stroke-width:0.25") # Output the scorelines separately
        layer.append(bgroup)
      
      else:
         pdstr2 = scorepath + pdstr2
         self.drawline(str(pdstr2),'backing',layer,sstr=None) #output the back piece      
      return 


    def effect(self):
        layer = self.svg.get_current_layer()
        scale = self.svg.unittouu('1'+self.options.unit)
        polysides = int(self.options.polysides)
        poly1size = float(self.options.poly1size) * scale
        poly2size = float(self.options.poly2size) * scale
        halfpoly = self.options.halfpoly
        polylimit = polysides
        collarht = float(self.options.collarheight) * scale
        partcnt = int(self.options.collarparts)
        tab_angle = float(self.options.tabangle)
        tab_height = float(self.options.tabheight) * scale
        dashlength = float(self.options.dashlength) * scale
        lines_on_wrapper = self.options.linesonwrapper
        circumscribed = self.options.circumscribed
        if not circumscribed:
            poly1size = poly1size/math.cos(math.pi/polysides)
            poly2size = poly2size/math.cos(math.pi/polysides)
        polylarge = max(poly1size, poly2size) # Larger of the two polygons
        polysmall = min(poly1size, poly2size) # Smaller of the two polygons
        polysmallR = polysmall/2
        polysmallr = polysmallR*math.cos(math.pi/polysides)
        polysmalltabht = tab_height
        
        if polysmallr < polysmalltabht:
           polysmalltabht = polysmallr
           
        if (halfpoly) and  ((polysides % 2)==0):
           polylimit = polysides // 2
        wpaths = []
        done = 0
        # We go through this loop twice
        # First time for the wrapper / decorative strip
        # Second time for the model, scorelines, and the lids
        while done < 2:
          w1 = (polylarge)*(math.sin(math.pi/polysides))
          w2 = (polysmall)*(math.sin(math.pi/polysides))
          if done == 0:
             # First time through, init the storage areas
             pieces = []
             nodes = []
             nd = Path()
             for i in range(4):
                 if i == 0:
                     nd.extend([Move(0.0,0.0)])
                 else:
                     nd.extend([Line(0.0,0.0)])
             nd.extend([ZoneClose()])
          else:
             # Second time through, empty the storage areas
             i = 0
             #SUECHANGE
             while i < polylimit:
                j = 0
                while j < 4:
                   del pieces[i][0]
                   j = j + 1
                i = i + 1
             i = 0
             while len(pieces) > 0:
                del pieces[0]
                i = i + 1
             i = 0
             while i < 4:
                del nodes[0]
                i = i + 1
          #SUECHANGE
          for pn in range(polylimit):
             nodes.clear()
             #what we need here is to skip the rotatation and just move
             # the x and y if there is no difference between the polygon sizes.
             #Added by Sue to handle equal polygons
             if poly1size == poly2size:
                nd[0] = Move(pn * w1,collarht)
                #nd[0].x =  pn * w1
                #nd[0].y = collarht
                nd[1] = Line(nd[0].x + w1,nd[0].y)
                #nd[1].x = nd[0].x + w1  
                #nd[1].y = nd[0].y
                nd[2] = Line(nd[1].x,nd[0].y - collarht)
                #nd[2].x = nd[1].x
                #nd[2].y = nd[0].y - collarht
                nd[3] = Line(nd[0].x,nd[2].y)
                #nd[3].x = nd[0].x  
                #nd[3].y = nd[2].y 
             else:
                if pn == 0:
                   nd[3] = Line(-w2/2,(polysmall/2)*math.cos(math.pi/polysides))
                   #nd[3].x = -w2/2
                   #nd[3].y = (polysmall/2)*math.cos(math.pi/polysides)
                   nd[0] = Move(-w1/2,(polylarge/2)*math.cos(math.pi/polysides))
                   #nd[0].x = -w1/2
                   #nd[0].y = (polylarge/2)*math.cos(math.pi/polysides)
                   vlen = math.sqrt(collarht**2 + (nd[0].y-nd[3].y)**2)
                   nd[0] = Move(-w1/2,nd[0].y + (vlen-(nd[0].y-nd[3].y)))
                   #nd[0].y = nd[0].y + (vlen-(nd[0].y-nd[3].y))
                   nd[2] = Line(w2/2,nd[3].y)
                   #nd[2].x = w2/2
                   #nd[2].y = nd[3].y
                   nd[1] = Line(w1/2,nd[0].y)
                   #nd[1].x = w1/2
                   #nd[1].y = nd[0].y
                   ox,oy = self.findIntersection(nd[0].x,nd[0].y,nd[3].x,nd[3].y,nd[1].x,nd[1].y,nd[2].x,nd[2].y)
                   Q2 = math.degrees(math.atan((nd[0].y - oy)/(w1/2 - ox)))
                   Q1 = 90 - Q2
                else:
                   nd.rotate(-2*Q1, (ox,oy), inplace=True)
             for i in range(4):
                nodes.append(copy.deepcopy(nd[i]))
             pieces.append(copy.deepcopy(nodes))
          if done == 0:
             for pc in range(partcnt):
                # Create the wrapper
                wpath = pathStruct() # We'll need this structure for makeTab
                wpath.id = "c1"
                dscores = Path()
                sidecnt = math.ceil(polysides/partcnt)
                if pc == partcnt - 1:
                   # Last time through creates the remainder of the pieces
                   #SUECHANGE
                   sidecnt = polylimit - math.ceil(polysides/partcnt)*pc
                startpc = pc*math.ceil(polysides/partcnt)
                endpc = startpc + sidecnt
                for pn in range(startpc, endpc):
                   # First half
                   if(pn == startpc):
                      ppt0 = Move(pieces[pn][0].x,pieces[pn][0].y)
                      # We're also creating wpath for later use in creating the model
                      wpath.path.extend([ppt0])
                   ppt1 = Line(pieces[pn][1].x,pieces[pn][1].y)
                   wpath.path.extend([ppt1])
                   if pn < endpc - 1:
                      # Put scorelines across the collar
                      ppt2 = Line(pieces[pn][2].x,pieces[pn][2].y)
                      spaths = self.makescore(ppt1, ppt2,dashlength)
                      dscores += spaths
                for pn in range(endpc-1, startpc-1, -1):
                   # Second half
                   if(pn == (endpc-1)):
                      ppt2 = Line(pieces[pn][2].x,pieces[pn][2].y)
                      wpath.path.extend([ppt2])
                   ppt3 = Line(pieces[pn][3].x,pieces[pn][3].y)
                   wpath.path.extend([ppt3])
                wpath.path.extend([ZoneClose()])
                wpaths.append(copy.deepcopy(wpath)) # Hold onto the path for the next step
                if math.isclose(dashlength, 0.0):
                    if lines_on_wrapper:
                        group = Group()
                        group.label = 'group'+str(pc)+'ws'
                        self.drawline(str(wpath.path),'wrapper'+str(pc),group,sstr="fill:#ffdddd;stroke:#000000;stroke-width:0.25") # Output the wrapper
                        self.drawline(str(dscores),'wscore'+str(pc)+'w',group,sstr=None) # Output the scorelines separately
                        layer.append(group)
                    else:
                        self.drawline(str(wpath.path),'wrapper'+str(pc),layer,sstr="fill:#ffdddd;stroke:#000000;stroke-width:0.25") # Output the wrapper
                else:
                    if lines_on_wrapper:
                        wpath.path = dscores + wpath.path # Output scorelines with wrapper
                    self.drawline(str(wpath.path),'wrapper'+str(pc),layer,sstr="fill:#ffdddd;stroke:#000000;stroke-width:0.25") # Output the wrapper
                while len(wpath.path) > 0:
                    del wpath.path[0]
             done = 1
          else:
             # Create the model
             for pc in range(partcnt):
                dprop = Path()
                dscores = Path()
                #SUECHANGE
                sidecnt = math.ceil(polylimit/partcnt)
                if pc == partcnt - 1:
                   #SUECHANGE
                   sidecnt = polylimit- math.ceil(polylimit/partcnt)*pc
                #SUECHANGE
                startpc = pc*math.ceil(polylimit/partcnt)
                endpc = startpc + sidecnt
                for pn in range(startpc, endpc):
                   # First half
                   if pn == startpc:
                      dprop.extend([Move(pieces[pn][0].x,pieces[pn][0].y)])
                   cpt1 = Move(pieces[pn][0].x, pieces[pn][0].y)
                   cpt2 = Move(pieces[pn][1].x, pieces[pn][1].y)
                   tabpt1, tabpt2 = self.makeTab(wpaths[pc], cpt1, cpt2, tab_height, tab_angle)
                   dprop.extend([tabpt1])
                   dprop.extend([tabpt2])
                   dprop.extend([Line(pieces[pn][1].x,pieces[pn][1].y)])
                   # As long as we're here, create a scoreline along the tab...
                   spaths = self.makescore(pieces[pn][0], pieces[pn][1],dashlength)
                   dscores.extend(spaths)
                   # ...and across the collar
                   spaths = self.makescore(pieces[pn][1], pieces[pn][2],dashlength)
                   dscores.extend(spaths)
                for pn in range(endpc-1, startpc-1, -1):
                   # Second half
                   if(pn == (endpc-1)):
                      # Since we're starting on the last piece, put a tab on the end of it, too
                      cpt1 = Move(pieces[pn][1].x, pieces[pn][1].y)
                      cpt2 = Move(pieces[pn][2].x, pieces[pn][2].y)
                      tabpt1, tabpt2 = self.makeTab(wpaths[pc], cpt1, cpt2, tab_height, tab_angle)
                      dprop.extend([tabpt1])
                      dprop.extend([tabpt2])
                      # Create a scoreline along the tab
                      #spaths = self.makescore(pieces[pn][1], pieces[pn][2],dashlength)
                      #dscores.append(spaths)
                   dprop.extend([Line(pieces[pn][2].x,pieces[pn][2].y)])
                   cpt1 = Move(pieces[pn][2].x, pieces[pn][2].y)
                   cpt2 = Move(pieces[pn][3].x, pieces[pn][3].y)
                   tabpt1, tabpt2 = self.makeTab(wpaths[pc], cpt1, cpt2, polysmalltabht, tab_angle)
                   dprop.extend([tabpt1])
                   dprop.extend([tabpt2])
                   dprop.extend([Line(pieces[pn][3].x,pieces[pn][3].y)])
                   # Create a scoreline along the tab
                   spaths = self.makescore(pieces[pn][2], pieces[pn][3],dashlength)
                   dscores.extend(spaths)
                dprop.extend([ZoneClose()])
                if math.isclose(dashlength, 0.0):
                   group = Group()
                   group.label = 'group'+str(pc)+'ms'
                   self.drawline(str(dprop),'model'+str(pc),group,sstr=None) # Output the model
                   self.drawline(str(dscores),'score'+str(pc)+'m',group,sstr=None) # Output the scorelines separately
                   layer.append(group)
                else:
                   # lump together all the score lines with the model
                   dprop = dscores + dprop
                   self.drawline(dprop,'model'+str(pc),layer,sstr=None) # Output the model

             # At this point, we can generate the top and bottom polygons
             # r = sidelength/(2*sin(PI/numpoly))
             #SUECHANGE 2 lines
             if polylimit<polysides:
                polylimit2 = polylimit+1
             else:
                polylimit2 = polylimit
             self.drawline(str(self.makepoly(w1, polysides,polylimit2)),'biglid',layer,sstr=None) # Output the bigger polygon
             self.drawline(str(self.makepoly(w2, polysides,polylimit2)),'smalllid',layer,sstr=None) # Output the smaller polygon
             if (polysides > polylimit):
                 self.makeback(w1,w2, polysides,polylimit2,collarht,tab_height,tab_angle,dashlength)   #output the back piece
             done = 2

if __name__ == '__main__':
    Collar().run()
