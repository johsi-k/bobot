#
# Digital World 2D
# Maze Level 2 (Actual)
#
# 15FO1 Group 1
# Daniel Yong Kaijie
# Heng Xian Jing
# Jesandry
# Keong Jo Hsi
# Soo Kai Leng
#

import json
import libdw.sm as sm
import libdw.gfx as gfx
import libdw.util as util
import libdw.eBotsonarDist as sonarDist
import math
import threading
import time
import urllib2
from soar.io import io

######################################################################
#
#            Functions
#
######################################################################

#
# params x (string): input URL that contains information about exposure areas and no. of plates
# return (list of strings): list of stations in sequence
#

def readURL(x):
    output = []
    dataList = urllib2.urlopen(urllib2.Request(x, headers={'User-Agent': 'SUTD 2D Demo'})).read(20000).split()
    for i in range(0, len(dataList)/2):
        letter = dataList[i*2]
        number = int(dataList[i*2+1])
        output += ['X', letter] * int(math.ceil(number/6.0))
    return output + ['X']

#
# params x (list of strings): list of stations
# return (list of strings): list of directions
#

def pathList(x):  # x is a list
    output = ['F', 'F']
    for i in range(0, len(x)-1):  # ends at 2nd-last element
        thisElement = x[i]
        nextElement = x[i+1]
        if thisElement == 'X':
            if nextElement == 'A':
                output += ['R', 'R']
            elif nextElement == 'B':
                output += ['R', 'F']
            elif nextElement == 'C':
                output += ['F', 'R', 'F']
            elif nextElement == 'D':
                output += ['F', 'R', 'L']
        elif thisElement == 'A':
            output += ['L', 'L']
        elif thisElement == 'B':
            output += ['F', 'L']
        elif thisElement == 'C':
            output += ['F', 'L', 'F']
        elif thisElement == 'D':
            output += ['R', 'L', 'F']
    return output

#
# params string (string): output string to be written into file
#

def writeFile(string):
    f = open("test2d.txt", 'w')
    f.write(string)
    f.close()

######################################################################
#
#            CLASSES
#
######################################################################

#
# description: firebase application which contains put, post and get methods
# params url (string): Firebase URL
#        token (string): Firebase token
#

class FirebaseApplication():
    def __init__(self, url, token):
        self.url = url
        self.firebaseToken = token

    def put(self, root, node, data):
        json_url = self.url + root + node
        opener = urllib2.build_opener(urllib2.HTTPHandler)
        request = urllib2.Request(json_url+'.json?auth='+self.firebaseToken,
			data = json.dumps(data))

        request.add_header('Content-Type', 'your/contenttype')
        request.get_method = lambda: 'PUT'
        result = opener.open(request)
        if result.getcode() == 200:
            return "OK"
        else:
            return "ERROR"

    def post(self, newnode, data):
        json_url = self.url + newnode
        opener = urllib2.build_opener(urllib2.HTTPHandler)
        request = urllib2.Request(json_url + '.json?auth=' + self.firebaseToken,
            data = json.dumps(data))

        request.add_header('Content-Type', 'your/contenttype')
        request.get_method = lambda: 'POST'
        result = opener.open(request)
        if result.getcode() == 200:
            return "OK"
        else:
            return "ERROR"

    def get(self, node):
        json_url = self.url + node + '.json'
        response = urllib2.urlopen(json_url)
        status = json.loads(response.read())
        return status

#
# description: allows put method of FirebaseApplication to be called in a separate thread
# params station (string)
#        ldr (list of strings) ;  ldr input from ebot is a list of strings
#        temp (int)
#

class writeData(threading.Thread):
    def __init__(self, station, ldr, temp):
        threading.Thread.__init__(self)
        self.station = station
        self.ldr = ldr
        self.temp = temp

    def run(self):
        fb.put("/station%s/"%(self.station),"ldr", self.ldr[0])
        fb.put("/station%s/"%(self.station),"temp", self.temp)
        fb.put("/station%s/"%(self.station),"time", time.strftime("%H:%M:%S|%d/%m/%y", time.localtime()))

######################################################################
#
#            Brain SM
#
######################################################################

stationList = readURL("http://people.sutd.edu.sg/~oka_kurniawan/10_009/y2015/2d/tests/level1_2.inp")
fb = FirebaseApplication("https://bobot.firebaseio.com", "7Ml97UKdUTK9zy82aNVMN0zYRwMLZKlMcAJ6msrS")
path = pathList(stationList)

###################################
#            variables
###################################

desiredRight = 0.65
forwardVelocity = 0.1
currentTheta = 0.0
finalTheta = 0.0
turnForwardVel = 0.09
turnRvel = 0.2
startTime = 0
junctionHistory = [False] * 5
deadEndHistory = [False] * 5
sensorHistory = []
outputString = ''
prevE = 1.5

for i in range(5):
    sensorHistory.append([0]*5)

print stationList
print path

############# STATES #############################
#
# Starting state: moveForward
#
# moveForward: junction, sendData, moveForward
#
# junction: straightAhead, turnLeft, turnRight
#
# straightAhead: moveForward, straightAhead
# turnLeft: moveForward, turnLeft
# turnRight: moveForward, turnRight
#
# sendData: deadEnd
#
# deadEnd: turn180, stop
#
# stop: stop
#
##################################################

class MySMClass(sm.SM):

    startState = 'startState'

    def getNextValues(self, state, inp):
        global prevE, path, currentTheta, finalTheta, turnForwardVel, \
            turnRvel, startTime, junctionHistory, deadEndHistory, outputString, sensorHistory, stationList

        # Adjusts inp.sonars values to account for error spikes
        # saves the last 5 inp.sonars readings, and takes the average (ignoring max and min values)
        for i in range(5):
            sensorHistory[i].pop(0)
            sensorHistory[i].append(inp.sonars[i])
            inp.sonars[i] = (sum(sensorHistory[i]) - max(sensorHistory[i]) - min(sensorHistory[i])) / 3.0

        temp = inp.temperature
        ldr = inp.light

        # use k1 and k2 values to calculate rvel
        k1 = 30
        k2 = -29.7
        v = sonarDist.getDistanceRight(inp.sonars)
        e = desiredRight - v
        angvel = k1*e + k2*prevE

        # cap rvel at 0.4 or -0.4
        cap = 0.4
        if angvel > cap:
            angvel = cap
        elif angvel < -cap:
            angvel = -cap

        # set default output to wallfollower values
        output = io.Action(forwardVelocity,angvel)
        prevE = e

        # junction conditions
        juncLimit = 1.2
        cond1 = inp.sonars[0] >= 0.7 and inp.sonars[1] >= juncLimit and inp.sonars[3] >= juncLimit and inp.sonars[4] >= 0.7
        cond2 = inp.sonars[0] >= juncLimit and inp.sonars[1] >= juncLimit and inp.sonars[2] >= juncLimit and inp.sonars[3] >= 0.7
        cond3 = inp.sonars[1] >= 0.7 and inp.sonars[2] >= juncLimit and inp.sonars[3] >= juncLimit and inp.sonars[4] >= juncLimit
        junction = cond1 or cond2 or cond3 # junction = True if condition is met
        junctionHistory.pop(0)
        junctionHistory = junctionHistory + [junction]
        didEnterJunction = True
        for i in junctionHistory:
            didEnterJunction = didEnterJunction and i

        # deadend conditions
        limitFC = 0.6
        deadEnd = inp.sonars[1] <= limitFC and inp.sonars[2] <= limitFC and inp.sonars[3] <= limitFC  # deadEnd = True if condition is met
        deadEndHistory.pop(0)
        deadEndHistory = deadEndHistory + [deadEnd]
        didEnterDeadEnd = True
        for i in deadEndHistory:
            didEnterDeadEnd = didEnterDeadEnd and i

###################################
#            startState
###################################

        if state == 'startState':

            startTime = time.time()
            nextState = 'moveForward'
            output = io.Action(0, 0)

###################################
#            moveForward
###################################

        elif state == 'moveForward':

            if didEnterJunction:  # condition satisfied only when junction is met 5 times in a row
                nextState = 'junction'
                output = io.Action(0, 0)

            elif didEnterDeadEnd:  # condition satisfied only when deadEnd is met 5 times in a row
                startTime = time.time()
                nextState = 'sendData'
                output = io.Action(0, 0)

            else:
                nextState = 'moveForward'

                if inp.sonars[3] >= 2.5 and inp.sonars[1] >= 2.5 and inp.sonars[2] >= 1:
                    output = io.Action(forwardVelocity, 0)

                elif time.time() - startTime < 1:
                    output = io.Action(0, 0)

###################################
#            junction
###################################

        elif state == 'junction':

            currentTheta = inp.odometry.theta

            if path[0] == 'F':
                startTime = time.time()
                nextState = 'straightAhead'

            elif path[0] == 'L':
                finalTheta = util.fixAnglePlusMinusPi(currentTheta + math.pi/2)
                nextState = 'turnLeft'

            elif path[0] == 'R':
                finalTheta = util.fixAnglePlusMinusPi(currentTheta - math.pi/2 + 0.25)
                nextState = 'turnRight'

            output = io.Action(0, 0)

###################################
#            turnLeft
###################################

        elif state == 'turnLeft':

            if util.nearAngle(finalTheta, inp.odometry.theta, 0.05):
                startTime = time.time() - 5
                nextState = 'straightAhead'
                output = io.Action(0, 0)

            else:  # turn completed
                nextState = 'turnLeft'
                output = io.Action(turnForwardVel, turnRvel)


###################################
#            turnRight
###################################

        elif state == 'turnRight':

            if util.nearAngle(finalTheta, inp.odometry.theta, 0.05):
                startTime = time.time() - 5
                nextState = 'straightAhead'
                output = io.Action(0, 0)

            else:
                nextState = 'turnRight'
                output = io.Action(turnForwardVel, -turnRvel)

###################################
#            straightAhead
###################################

        elif state == 'straightAhead':

            if time.time() - startTime < 10.0:
                nextState = 'straightAhead'
                output = io.Action(0.1, 0.03)

            else:
                path.pop(0)
                nextState = 'moveForward'

###################################
#            sendData
###################################

        elif state == 'sendData':

            if len(stationList) == 1:
                word = 'Finished, and arrived at '
            elif stationList[0] == 'X':
                word = 'Collect Plates at '
            else:
                word = 'Expose Plates at '

            wtime = time.strftime("%H:%M:%S", time.localtime())
            wdate = time.strftime("%d-%m-%Y", time.localtime())
            outputString += "<%s> || <%s> || %s%s \n"%(wtime, wdate, word, stationList[0])
            writeFile(outputString)

            if stationList[0] != 'X':
                wd = writeData(stationList[0], ldr, temp)
                wd.start()

            stationList.pop(0)  # X has to be popped also
            nextState = 'deadEnd'
            output = io.Action(0, 0)


###################################
#            deadEnd
###################################

        elif state == 'deadEnd':

            if len(path) == 0:
                nextState = 'stop'

            else:
                if time.time() - startTime < 7.0:
                    nextState = 'deadEnd'
                else:
                    currentTheta = inp.odometry.theta
                    finalTheta = util.fixAnglePlusMinusPi(currentTheta - math.pi + 0.5)
                    nextState = 'turn180'

            output = io.Action(0, 0)

###################################
#            turn180
###################################

        elif state == 'turn180':

            if util.nearAngle(finalTheta, inp.odometry.theta, 0.05):
                nextState = 'startState'
                output = io.Action(0, 0)

            else:
                nextState = 'turn180'
                output = io.Action(0, -0.5)

###################################
#            stop
###################################

        elif state == 'stop':
            nextState = 'stop'
            output = io.Action(0, 0)

###################################

        print "                       %.4f                       \n"%(inp.sonars[2])
        print "           %.4f                       %.4f           \n"%(inp.sonars[1], inp.sonars[3])
        print "%.4f                                              %.4f\n\n\n"%(inp.sonars[0], inp.sonars[4])
        print cond1, cond2, cond3, deadEnd
        print state
        print output
        print stationList
        print path

        return nextState , output


# Your code here


mySM = MySMClass()
mySM.name = 'brainSM'


######################################################################
#
#            Running the robot
#
######################################################################

def setup():
    robot.gfx = gfx.RobotGraphics(drawSlimeTrail=False)
    robot.gfx.addStaticPlotSMProbe(y=('rightDistance', 'sensor',
                                      'output', lambda x:x))
    robot.behavior = mySM
    robot.behavior.start(traceTasks = robot.gfx.tasks())

def step():
    robot.behavior.step(io.SensorInput()).execute()
    io.done(robot.behavior.isDone())

def brainStop():
    pass
