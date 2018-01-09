author = 'avs2368'

"""
file: Terrain.py
language: python3
author: avs2368@g.rit.edu   (Abhishek Shakwala)

description: Generating optimal paths for orienteering during different seasons.
             Given a map with terrain information, elevation contours, and a set
             or sequence of locations to visit ("controls").
"""

from PIL import Image
from queue import PriorityQueue
from math import sqrt
import time

LATITUDE = 7.55
LONGITUDE = 10.29

COLUMNS = 395
ROWS = 500

walkablePath = []
unwalkablePath = []
openLand = []
roughMeadow = []
easyMovementForest = []
slowRunForest = []
walkForest = []
impassibleVegetation = []
lakeMarsh = []
pavedRoad = []
footPath = []
outOfBound = []
final = []
image = 'terrain.png'
drawPaths = []

class Node:
    def __init__(self, x, y, terrainSpeed, elevation, parent, f = 0, g = 0, h = 0):
        self.x = x
        self.y = y
        self.parent = None
        self.terrainSpeed = terrainSpeed
        self.elevation = elevation
        self.f = f
        self.g = g
        self.h = h

def drawPath(finalPath, image):
    final = []
    image = Image.open(image)
    image = image.convert(mode="RGB")
    for paths in finalPath:
        for path in paths:
            final.append(path)
    for path in final:
        image.putpixel((path[1], path[0]), (255, 0, 0))
    image.show()

def drawICEMap(ice, image):
    image = Image.open(image)
    image = image.convert(mode="RGB")
    for i in ice:
        image.putpixel((i[1], i[0]), (100, 200, 250))
    image.save("iceMap.png")

def drawMud(mudPixel, image):
    image = Image.open(image)
    image = image.convert(mode="RGB")
    for i in mudPixel:
        image.putpixel((i[1], i[0]), (165,42,42))
    image.save("mudMap.png")

def textOutput(finalPath):
    final = []
    for paths in finalPath:
        for path in paths:
            final.append(path)
    for path in final:
        print(path)

def heuristic(start, dest):
    return sqrt((start.x - dest.x) ** 2 + (start.y - dest.y) ** 2 + (start.elevation - dest.elevation) ** 2) / 2

def getNeighbors(current, map):
    neighbors = []
    y = current.y
    x = current.x - 1
    if -1 < x < ROWS and -1 < y < COLUMNS:
        neighbor = map[x][y]
        neighbor.distance = LATITUDE
        if neighbor.terrainSpeed != 0:
            neighbors.append(neighbor)
    y = current.y - 1
    x = current.x
    if -1 < x < ROWS and -1 < y < COLUMNS:
        neighbor = map[x][y]
        neighbor.distance = LONGITUDE
        if neighbor.terrainSpeed != 0:
            neighbors.append(neighbor)
    y = current.y + 1
    x = current.x
    if -1 < x < ROWS and -1 < y < COLUMNS:
        neighbor = map[x][y]
        neighbor.distance = LONGITUDE
        if neighbor.terrainSpeed != 0:
            neighbors.append(neighbor)
    y = current.y
    x = current.x + 1
    if -1 < x < ROWS and -1 < y < COLUMNS:
        neighbor = map[x][y]
        neighbor.distance = LATITUDE
        if neighbor.terrainSpeed != 0:
            neighbors.append(neighbor)
    return neighbors

def costFunction(start, end):
    distance2d = end.distance
    terrainDelta = ((start.terrainSpeed / 60) + (end.terrainSpeed / 60))/2
    elevationDelta = (start.elevation - end.elevation)
    distance3d = sqrt(distance2d ** 2 + elevationDelta ** 2)
    time = distance3d / (terrainDelta + elevationDelta / 40)
    '''
    if distance < 0:
        print("Cost is negative!!!", distance)
    '''
    return time

def doAStar(start, goal, map, final):          #walkable and unwalkable deleted from parameters
    openlist = set()
    closedlist = set()
    current = start
    openlist.add(current)
    while len(openlist) >= 0:
        current = min(openlist, key=lambda o:o.f)
        if current.x == goal.x and current.y == goal.y:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            for paths in path:
                final.append([paths.x, paths.y])
            heu = heuristic(start, goal)
            return final[::-1]
        openlist.remove(current)
        closedlist.add(current)
        for neighbor in getNeighbors(current, map):
            if neighbor in closedlist:
                continue
            if neighbor in openlist:
                g = current.g + costFunction(current, neighbor)
                if neighbor.g > g:
                    neighbor.parent = current
                    neighbor.g = g
            else:
                neighbor.parent = current
                neighbor.g = current.g + costFunction(current, neighbor)
                neighbor.h = heuristic(neighbor, goal)
                neighbor.f = neighbor.g + neighbor.h
                openlist.add(neighbor)

def fallSeason(controls, pixelInfo):
    print("Fall")

    ######### DEFINING SPEED FOR TERRAIN TYPE IN FALL #########

    sol = 70
    srm = 30
    semf = 35
    ssrf = 20
    swf = 15
    siv = 0
    slsm = 0
    spr = 80
    sfp = 75
    soob = 0

    terrainType = {1: sol, 2: srm, 3: semf, 4: ssrf, 5: swf, 6: siv, 7: slsm, 8: spr, 9: sfp, 10: soob}

    map = list()
    tempmap = list()
    for i in range(ROWS):
        for j in range(COLUMNS):
            tempmap.append(Node(pixelInfo[i][j][2], pixelInfo[i][j][3], terrainType.get(pixelInfo[i][j][0]), pixelInfo[i][j][1], None))
        map.append(tempmap)
        tempmap = list()
    while len(controls) >= 4:
        #doAStar([controls[0], controls[1]], [controls[2], controls[3]], pixelInfo, walkablePath, unwalkablePath)
        final = doAStar(map[controls[1]][controls[0]], map[controls[3]][controls[2]], map, [])
        map = list()
        tempmap = list()
        for i in range(ROWS):
            for j in range(COLUMNS):
                tempmap.append(Node(pixelInfo[i][j][2], pixelInfo[i][j][3], terrainType.get(pixelInfo[i][j][0]),
                                    pixelInfo[i][j][1], None))
            map.append(tempmap)
            tempmap = list()
        print(final)
        drawPaths.append(final)
        controls = controls[2:]
    drawPath(drawPaths, image)

def springSeason(controls, pixelInfo):
    print("Spring")

    ######### DEFINING SPEED FOR TERRAIN TYPE IN SPRING #########

    sol = 70
    srm = 30
    semf = 55
    ssrf = 25
    swf = 18
    siv = 0
    slsm = -1
    spr = 80
    sfp = 75
    soob = 0

    terrainType = {1: sol, 2: srm, 3: semf, 4: ssrf, 5: swf, 6: siv, 7: slsm, 8: spr, 9: sfp, 10: soob}

    map = list()
    tempmap = list()
    for i in range(ROWS):
        for j in range(COLUMNS):
            tempmap.append(
                Node(pixelInfo[i][j][2], pixelInfo[i][j][3], terrainType.get(pixelInfo[i][j][0]), pixelInfo[i][j][1],
                     None))
        map.append(tempmap)
        tempmap = list()

    mudPixel = []

    for i in range(ROWS):
        for j in range(COLUMNS):
            if map[i][j].terrainSpeed != -1:
                for z in range(1, 16):
                    if i + z < 500:
                        elev = map[i + z][j].elevation - map[i][j].elevation
                        if map[i + z][j].terrainSpeed == -1 and elev < 1:
                            map[i + z][j].terrainSpeed = 0
                            mudPixel.append([i + z, j])
                    if i - z > 0:
                        elev = map[i - z][j].elevation - map[i][j].elevation
                        if map[i - z][j].terrainSpeed == -1 and elev < 1:
                            map[i - z][j].terrainSpeed = 0
                            mudPixel.append([i - z, j])

                    if j + z < 395:
                        elev = map[i][j + z].elevation - map[i][j].elevation
                        if map[i][j + z].terrainSpeed == -1 and elev < 1:
                            map[i][j + z].terrainSpeed = 0
                            mudPixel.append([i, j + z])
                    if j - z > 0:
                        elev = map[i][j - z].elevation - map[i][j].elevation
                        if map[i][j - z].terrainSpeed == -1 and elev < 1:
                            map[i][j - z].terrainSpeed = 0
                            mudPixel.append([i, j - z])

    for i in range(ROWS):
        for j in range(COLUMNS):
            if map[i][j].terrainSpeed == -1:
                map[i][j].terrainSpeed = 0

    drawMud(mudPixel, image)

    while len(controls) >= 4:
        #doAStar([controls[0], controls[1]], [controls[2], controls[3]], pixelInfo, walkablePath, unwalkablePath)
        final = doAStar(map[controls[1]][controls[0]], map[controls[3]][controls[2]], map, [])
        map = list()
        tempmap = list()
        for i in range(ROWS):
            for j in range(COLUMNS):
                tempmap.append(Node(pixelInfo[i][j][2], pixelInfo[i][j][3], terrainType.get(pixelInfo[i][j][0]),
                                    pixelInfo[i][j][1], None))
            map.append(tempmap)
            tempmap = list()
        print(final)
        drawPaths.append(final)
        controls = controls[2:]
    drawPath(drawPaths, "mudMap.png")

def winterSeason(controls, pixelInfo, flatList):
    print("Winter")

    ######### DEFINING SPEED FOR TERRAIN TYPE IN WINTER #########

    sol = 60
    srm = 20
    semf = 35
    ssrf = 15
    swf = 10
    siv = 0
    slsm = 0
    spr = 70
    sfp = 55
    soob = 0

    terrainType = {1: sol, 2: srm, 3: semf, 4: ssrf, 5: swf, 6: siv, 7: slsm, 8: spr, 9: sfp, 10: soob}

    edgeOfWater = []
    for i in flatList:
        if i[0] == 7:
            edgeOfWater.append(i)

    map = list()
    tempmap = list()
    for i in range(ROWS):
        for j in range(COLUMNS):
            tempmap.append(
                Node(pixelInfo[i][j][2], pixelInfo[i][j][3], terrainType.get(pixelInfo[i][j][0]), pixelInfo[i][j][1],
                     None))
        map.append(tempmap)
        tempmap = list()
    wateredge = []
    waterpixel = []
    icePixel = set()
    prevx = -1
    prevy = -1
    line1 = False
    lastLine = False

    ############# GETTING THE EDGE OF THE LAKE/ SWAMP/ MARSH ##############

    iceEdge = set()
    for edge in edgeOfWater:
        if edge[2] + 1 < 500:
            if map[edge[2] + 1][edge[3]].terrainSpeed != 0:
                iceEdge.add(map[edge[2]][edge[3]])
        if map[edge[2] - 1][edge[3]].terrainSpeed != 0:
            iceEdge.add(map[edge[2]][edge[3]])
        if edge[3] + 1 < 395:
            if map[edge[2]][edge[3] + 1].terrainSpeed != 0:
                iceEdge.add(map[edge[2]][edge[3]])
        if map[edge[2]][edge[3] - 1].terrainSpeed != 0:
            iceEdge.add(map[edge[2]][edge[3]])

    ############## CALCULATING THE ICE PIXEL FROM WATER EDGE PIXEL ################

    icePixel = set()
    for wp in iceEdge:
        map[wp.x][wp.y].terrainSpeed = 50
        for i in range(1,8):
            if wp.x + i < 500:
                if map[wp.x + i][wp.y].terrainSpeed == 0:
                    map[wp.x + i][wp.y].terrainSpeed = 50
                    icePixel.add(map[wp.x + i][wp.y])
                else:
                    break
        for i in range(1, 8):
            if wp.x - i < 500:
                if map[wp.x - i][wp.y].terrainSpeed == 0:
                    map[wp.x - i][wp.y].terrainSpeed = 50
                    icePixel.add(map[wp.x - i][wp.y])
                else:
                    break
        for i in range(1, 8):
            if wp.y + i < 395:
                if map[wp.x][wp.y + i].terrainSpeed == 0:
                    map[wp.x][wp.y + i].terrainSpeed = 50
                    icePixel.add(map[wp.x][wp.y + i])
                else:
                    break
        for i in range(1, 8):
            if wp.y - i < 395:
                if map[wp.x][wp.y - i].terrainSpeed == 0:
                    map[wp.x][wp.y - i].terrainSpeed = 50
                    icePixel.add(map[wp.x][wp.y - i])
                else:
                    break
    '''
    for wp in wateredge:
        if map[wp.x][wp.y].terrainSpeed == -1:
            map[wp.x][wp.y].terrainSpeed = 0
    '''

    ice = []
    for i in icePixel:
        ice.append([i.x, i.y])

    drawICEMap(ice, image)

    while len(controls) >= 4:
        #doAStar([controls[0], controls[1]], [controls[2], controls[3]], pixelInfo, walkablePath, unwalkablePath)
        final = doAStar(map[controls[1]][controls[0]], map[controls[3]][controls[2]], map, [])
        map = list()
        tempmap = list()
        for i in range(ROWS):
            for j in range(COLUMNS):
                tempmap.append(Node(pixelInfo[i][j][2], pixelInfo[i][j][3], terrainType.get(pixelInfo[i][j][0]),
                                    pixelInfo[i][j][1], None))
            map.append(tempmap)
            tempmap = list()
        print(final)
        drawPaths.append(final)
        controls = controls[2:]
    drawPath(drawPaths, "iceMap.png")

def summerSeason(controls, pixelInfo):
    print("Summer")

    ######### DEFINING SPEED FOR TERRAIN TYPE IN SUMMER #########

    sol = 70
    srm = 30
    semf = 55
    ssrf = 25
    swf = 18
    siv = 0
    slsm = 0
    spr = 80
    sfp = 75
    soob = 0

    terrainType = {1: sol, 2: srm, 3: semf, 4: ssrf, 5: swf, 6: siv, 7: slsm, 8: spr, 9: sfp, 10: soob}

    map = list()
    tempmap = list()
    for i in range(ROWS):
        for j in range(COLUMNS):
            tempmap.append(Node(pixelInfo[i][j][2], pixelInfo[i][j][3], terrainType.get(pixelInfo[i][j][0]), pixelInfo[i][j][1], None))
        map.append(tempmap)
        tempmap = list()
    while len(controls) >= 4:
        #doAStar([controls[0], controls[1]], [controls[2], controls[3]], pixelInfo, walkablePath, unwalkablePath)
        final = doAStar(map[controls[1]][controls[0]], map[controls[3]][controls[2]], map, [])
        map = list()
        tempmap = list()
        for i in range(ROWS):
            for j in range(COLUMNS):
                tempmap.append(Node(pixelInfo[i][j][2], pixelInfo[i][j][3], terrainType.get(pixelInfo[i][j][0]),
                                    pixelInfo[i][j][1], None))
            map.append(tempmap)
            tempmap = list()
        print(final)
        drawPaths.append(final)
        controls = controls[2:]
    drawPath(drawPaths, image)
    #textOutput(drawPaths)

def inputReader(image, elevation, whitePath, brownPath, redPath, season):
    print("Inside Input Reader")
    xyCoordinate = []
    elevationData = []
    elevationPoints = []
    pixelInfo = []
    image = Image.open(image)
    image = image.convert(mode="RGB")
    imageData = list(image.getdata())
    imageData = [list(pixel) for pixel in imageData]
    with open(elevation) as ef:
        elevData = ef.readlines()
    for line in elevData:
        line = line.strip()
        line = line.split("   ")
        line = line[:-5]
        elevationData.append(line)
    for i in range(len(elevationData)):
        j = 0
        while j != 395:
            elevationPoints.append(float(elevationData[i][j]))
            j += 1
    pixel = 0
    x = 0
    y = 0
    mainMatrix = []
    matrix = []
    for i in range(image.size[1]):
        for j in range(image.size[0]):
            xyCoordinate.append([i, j])
            if imageData[pixel] == [248, 148, 18]:
                tt = 1
            elif imageData[pixel] == [255, 192, 0]:
                tt = 2
            elif imageData[pixel] == [255, 255, 255]:
                tt = 3
            elif imageData[pixel] == [2, 208, 60]:
                tt = 4
            elif imageData[pixel] == [2, 136, 40]:
                tt = 5
            elif imageData[pixel] == [5, 73, 24]:
                tt = 6
            elif imageData[pixel] == [0, 0, 255]:
                tt = 7
            elif imageData[pixel] == [71, 51, 3]:
                tt = 8
            elif imageData[pixel] == [0, 0, 0]:
                tt = 9
            elif imageData[pixel] == [205, 0, 101]:
                tt = 10
            pixelInfo.append([tt] + [float(elevationData[i][j])] + [i, j])
            matrix.append([tt] + [float(elevationData[i][j])] + [i, j])
            pixel += 1
        mainMatrix.append(matrix)
        matrix = []
    print("395 * 500 = ", pixel)
    #print(mainMatrix[100][150])
    flatList = pixelInfo
    pixelInfo = mainMatrix

    ################## TERRAIN TYPE #####################

    for i in range(image.size[1]):
        for j in range(image.size[0]):
            if pixelInfo[i][j][0] == 1:
                openLand.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 2:
                roughMeadow.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 3:
                easyMovementForest.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 4:
                slowRunForest.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 5:
                walkForest.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 6:
                impassibleVegetation.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 7:
                lakeMarsh.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 8:
                pavedRoad.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 9:
                footPath.append(pixelInfo[i])
            elif pixelInfo[i][j][0] == 10:
                outOfBound.append(pixelInfo[i])

    ###################### INPUT CONTROLS #####################

    whiteControl = []
    with open(whitePath) as wf:
        white = wf.readlines()
    for line in white:
        line = line.strip()
        line = line.split(' ')
        whiteControl.append(line)
    i = 0
    j = 0
    for xy in whiteControl:
        for z in xy:
            whiteControl[i][j] = int(z)
            j += 1
        i += 1
        j = 0
    wc = whiteControl
    whiteControl = []
    for elem in wc:
        for each in elem:
            whiteControl.append(each)
    brownControl = []
    with open(brownPath) as bf:
        brown = bf.readlines()
    for line in brown:
        line = line.strip()
        line = line.split(' ')
        brownControl.append(line)
    i = 0
    j = 0
    for xy in brownControl:
        for z in xy:
            brownControl[i][j] = int(z)
            j += 1
        i += 1
        j = 0
    bc = brownControl
    brownControl = []
    for elem in bc:
        for each in elem:
            brownControl.append(each)
    redControl = []
    with open(redPath) as rp:
        red = rp.readlines()
    for line in red:
        line = line.strip()
        line = line.split(' ')
        redControl.append(line)
    i = 0
    j = 0
    for xy in redControl:
        for z in xy:
            redControl[i][j] = int(z)
            j += 1
        i += 1
        j = 0
    rc = redControl
    redControl = []
    for elem in rc:
        for each in elem:
            redControl.append(each)
    print("Data Ready")
    timestart = time.time()
    if season == "summer":
        summerSeason(whiteControl, pixelInfo)
    elif season == "fall":
        fallSeason(whiteControl, pixelInfo)
    elif season == "winter":
        winterSeason(whiteControl, pixelInfo, flatList)
    elif season == "spring":
        springSeason(whiteControl, pixelInfo)
    timeend = time.time()
    #print("Total Time:", timeend - timestart)

if __name__ == '__main__':
    image = 'terrain.png'
    elevation = 'map.txt'
    whitePath = 'white.txt'
    brownPath = 'brown.txt'
    redPath = 'red.txt'
    season = input("Enter the season:")
    inputReader(image, elevation, whitePath, brownPath, redPath, season.lower())