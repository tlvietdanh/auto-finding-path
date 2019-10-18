from tkinter import *
from queue import Queue

import math

# Declare constant variables
PI = 3.14159265359
BORDER = -2
WALL_BORDER = -1
INSIDE_POLYGON = -0.5
START = 1
PATH = 3
ROAD = 2.5
END = 4
CATCH = 2
DISTANCE = 40
EMPTYPOINT = 0
MAX_WIDTH = 1200
MAX_HEIGHT = 900
COLOR = ['pink', 'yellow', 'purple']

class vertices:
    def __init__(self, x, y, type):
        self.x = x
        self.y = y
        self.type = type
        self.listAdjacency = []

    def addMovableCell(self, mCol, mRow):
        dirVertices = [
            [1, 0], [-1, 0], # right, left
            [0, 1], [0, -1], # up, down
            [1, 1], [1, -1],  # cross up left, right
            [-1, 1], [-1, -1] # cross down left, right
        ]
        for xdir, ydir in dirVertices:
            if (self.x + xdir < 1 or self.x + xdir > mCol - 1):
                continue
            if (self.y + ydir < 1 or self.y + ydir > mRow - 1):
                continue
            self.listAdjacency.append([self.x + xdir, self.y + ydir])
    
    def callPoint(self, mCol):
        return self.y * mCol + self.x
        
    def __eq__(self, obj):
        return self.x == obj.x and self.y == obj.y

# Read input file
def ReadFile():
    output = []
    cusor = 0
    with open('input.txt','r') as f:
        for line in f:
            arr  = re.findall(r"\d+",line)
            output.append(arr)
    # Get number of columns and number of rows
    mCol = int(output[cusor][0])
    mRow = int(output[cusor][1])
    cusor += 1
    # Get start point and end point
    startPoint = vertices(int(output[cusor][0]), int(output[cusor][1]), START)
    endPoint = vertices(int(output[cusor][2]), int(output[cusor][3]), END)
    # Get list catch point(if existed)
    listCatchPoint = []
    if (output[cusor].__len__() > 4):
        j = 4
        while j < output[cusor].__len__() - 1:
            catchPoint = vertices(int(output[cusor][j]), int(output[cusor][j + 1]), CATCH)
            listCatchPoint.append(catchPoint)
            j += 2
    cusor += 1
    # Get catch point (if existed)
    # Get number of polygons
    numberPolygon = int(output[cusor][0])
    cusor += 1

    # Get posision of each polygon
    i = 0
    listPolygons = []
    while (i < numberPolygon):
        k = 0
        tempArr = []
        while (k < len(output[cusor]) - 1):
            temp = vertices(int(output[cusor][k]), int(output[cusor][k + 1]), END + i + 1)
            k += 2
            tempArr.append(temp)
        i += 1
        cusor += 1
        listPolygons.append(tempArr)
        del tempArr
    return (startPoint, endPoint, listCatchPoint, listPolygons, mRow, mCol)

# Initialize map
def initGraph(mRow, mCol, listPolygons, startPoint, endPoint, listCatchPoint):
    graph = []
    for y in range(mRow):
        for x in range(mCol):
            v = vertices(x, y, EMPTYPOINT)
            # Check border
            if x < 1 or y < 1 or x == mCol or y == mRow:
                v.type = (BORDER)
            else:
                for catch in listCatchPoint:
                    if v == catch:
                        v.type = catch.type
                for polygon in listPolygons:
                    for vPolygon in polygon:
                        if v == vPolygon:
                            v.type = vPolygon.type
                if (v == startPoint):
                    v.type = startPoint.type
                elif v == endPoint:
                    v.type = endPoint.type
                v.addMovableCell(mCol, mRow)
            graph.append(v)
    for i in range(mCol + 1):
        v = vertices(i, mRow, BORDER)
        graph.append(v)
    for i in range(mRow + 1):
        v = vertices(mCol, i, BORDER)
        graph.append(v)
    return graph

# Check if vertice c lies on line ab
def isInsideLine(a, b, c):
    if (c.x <= max(a.x, b.x) and c.x >= min(a.x, b.x) and c.y <= max(a.y, b.y) and c.y >= min(a.y, b.y)):
        return True
    return False

# Check orientation between 3 vertices a,b,c
# The function will return 0 if these vertices are colinear
# return 1 if clockwise
# return 2 if counterclockwise
def orientation(a, b, c):
    value = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y)
    if value == 0:
        return 0
    elif value > 0:
        return 1
    else:
        return 2

# Get Min and Max x,y coordinates of polygon
def getMax(polygon, mCol, mRow):
    xMax = 0
    yMax = 0
    xMin = mCol + 1
    yMin = mRow + 1
    for p in polygon:
        xMax = max(xMax, p.x)
        xMin = min(xMin, p.x)
        yMax = max(yMax, p.y)
        yMin = min(yMin, p.y)
    return (xMax, yMax, xMin, yMin)           

# Return true of a,b,c are in the same plane, else return false
def samePlane(a, b, c):
    if (a.y <= b.y and c.y <= b.y) or ( a.y >= b.y and c.y >= b.y):
        return True
    else:
        return False

# Check all vertices inside the polygon and set it to a graph
def isInsidePolygon(listEdge, graph, polygon, mCol, mRow):
    if (polygon.__len__() < 3):
        return
    xMax, yMax, xMin, yMin = getMax(polygon, mCol, mRow)
    
    for vertice in graph:
        lastest = vertices(mCol, vertice.y, 0)
        count = 0
        if vertice.x < xMax and vertice.x > xMin and vertice.y < yMax and vertice.y > yMin and vertice.type == EMPTYPOINT:
            j = 0
            listVer = []
            while j < polygon.__len__():
                if vertice != polygon[j] and orientation(vertice, lastest, polygon[j]) == 0:
                    if isInsideLine(vertice, lastest, polygon[j]):
                        #check same plane:
                        nextVer = polygon[(j + 1) % polygon.__len__()]
                        preVer = polygon[polygon.__len__() - 1]
                        if (j - 1 >= 0):
                            preVer = polygon[j - 1]
                        if (samePlane(preVer, polygon[j], nextVer)):
                            count += 1
                            listVer.append(j)
                j += 1
            i = 0
            while i < listEdge.__len__():
                for temp in listEdge[i]:
                    if orientation(vertice, lastest, graph[temp]) == 0:
                        if isInsideLine(vertice, lastest, graph[temp]):
                            count += 1
                            break
                i += 1
        if count % 2 == 1:
            vertice.type = INSIDE_POLYGON

# Create walls
def createWalls(graph, listPolygons, mRow, mCol):
    result = []
    visited = [False for i in range(graph.__len__())]
    path = [False for i in range(graph.__len__())]

    for polygon in listPolygons:
        listEgde = []
        i = 0
        while i < polygon.__len__():
            tempArr = []
            # current vertice of polygon
            startVertice = polygon[i]
            # Next vertice of polygon
            nextVertice = polygon[ (i + 1) % polygon.__len__()]

            # Finding path from the curent vertice to the next vertive
            for j in range(graph.__len__()):
                visited[j] = False
                path[j] = -1
            q = Queue()
            visited[startVertice.callPoint(mCol)] = True
            q.put(startVertice.callPoint(mCol))
            while not q.empty():
                u = q.get()
                for x, y in graph[u].listAdjacency:
                    point = y * mCol + x
                    if not visited[point] and point not in tempArr and graph[point].type != WALL_BORDER:
                        visited[point] = True
                        q.put(point)
                        path[point] = u
                        if point == nextVertice.callPoint(mCol):
                            q.queue.clear()
            tempArr = findPath(startVertice.callPoint(mCol), nextVertice.callPoint(mCol), path, tempArr)
            listEgde.append(tempArr)
            i += 1
        #change graph
        for i in listEgde:
            for j in i:
                graph[j].type = WALL_BORDER
        isInsidePolygon(listEgde, graph, polygon, mCol, mRow)
        result.append(listEgde)
    return result

# Breath first search algorithm to find the shotest path
def BFS(s, f, graph, col, row):
    visited = [False for i in range(graph.__len__())]
    path = [False for i in range(graph.__len__())]
    for i in range(graph.__len__()):
        visited[i] = False
        path[i] = -1
    q = Queue()
    visited[s.callPoint(col)] = True
    q.put(s.callPoint(col))
    while not q.empty():
        u = q.get()
        for x, y in graph[u].listAdjacency:
            point = y * col + x
            if not visited[point] and graph[point].type <= END and graph[point].type >= EMPTYPOINT:
                visited[point] = True
                q.put(point)
                path[point] = u
    resultArr = []
    resultArr = findPath(s.callPoint(col), f.callPoint(col), path, resultArr)
    return resultArr

# Return the path to the list
def findPath(s, f, path, mlist):
    if s == f:
        if mlist.__len__() > 0 : mlist.pop()
        return mlist
    else:
        if path[f] == -1:
            return mlist
        else:
            mlist.append(path[f])
            return findPath(s, path[f], path, mlist)

# Get the min distance from point to the point of the listCatchPoint
def getMinDistance(listCatchPoint, point, visited, mCol, mRow):
    mMin = mCol * mRow
    index = -1
    for j in range(listCatchPoint.__len__()):
        dist = math.sqrt(pow(point.x - listCatchPoint[j].x, 2) + pow(point.y - listCatchPoint[j].y, 2))
        if dist <= mMin and not visited[j]:
            mMin = dist
            index = j
    return index

# Create a road to from start point to the goal
def createRoad(graph, listCatchPoint, startPoint, endPoint, mCol, mRow):
    i = 0
    roadArr = []
    if listCatchPoint.__len__() == 0:
        roadArr = BFS(startPoint, endPoint, graph, mCol, roadArr)
    else:
        visited = [False for q in range(listCatchPoint.__len__() + 1)]
        start = startPoint
        
        while i <= listCatchPoint.__len__():
            temp = []
            index = getMinDistance(listCatchPoint, start, visited, mCol, mRow)
            end = endPoint
            if i < listCatchPoint.__len__() or index != -1:
                end = listCatchPoint[index]
            temp = BFS(start, end, graph, mCol, mRow)
            i += 1
            visited[index] = True
            start = end
            roadArr.append(temp)
    for i in roadArr:
        for j in i:
            graph[j].type = ROAD

# Draw map by tkinter
def draw_board(canvas, graph, row, col):
    for i in range(-1, row):
        if i < row:
            canvas.create_text((i + 1) * DISTANCE + DISTANCE * 1.5, DISTANCE / 2, text=i + 1)
        for j in range(-1, col):
            canvas.create_text(DISTANCE / 2, (j+1) * DISTANCE + DISTANCE * 1.5, text=j + 1)

    for v in graph:
        coords = ((v.x + 1) * DISTANCE, (v.y+1) * DISTANCE, (v.x + 1) * DISTANCE + DISTANCE, (v.y+1) * DISTANCE + DISTANCE)
        if (v.type == START):
            canvas.create_rectangle(coords, width=1, state='disabled', fill='red')
        elif (v.type == END):
            canvas.create_rectangle(coords, width=1, state='disabled', fill='blue')
        elif (v.type == CATCH):
            canvas.create_rectangle(coords, width=1, state='disabled', fill='SteelBlue3')    
        elif (v.type == BORDER):
            canvas.create_rectangle(coords, width=1, state='disabled', fill='gray')
        elif (v.type >= END + 1):
            canvas.create_rectangle(coords, width=1, state='disabled', fill='orange')
        elif (v.type == WALL_BORDER):
            canvas.create_rectangle(coords, width=1, state='disabled', fill='yellow')
        elif (v.type == ROAD):
            canvas.create_rectangle(coords, width=1, state='disable', fill='green')
        elif (v.type == EMPTYPOINT):
            canvas.create_rectangle(coords, width=1, state='disabled')                 

# main
def main():
    # Read data from the input file  
    startPoint, endPoint, listCatchPoint, listPolygons, mRow, mCol = ReadFile()

    # Initialize graph
    graph = initGraph(mRow, mCol, listPolygons, startPoint, endPoint, listCatchPoint)

    # Create Wall
    createWalls(graph, listPolygons, mRow, mCol)
    
    #find path from start to end
    createRoad(graph, listCatchPoint, startPoint, endPoint, mCol, mRow)
   
    # From this line, Those codes below will create an interface for user.
    # Create root view which is containing all views
    root = Tk()
    # Create frame which is containing canvas
    width = mCol * DISTANCE + 100
    height = mRow * DISTANCE + 100
    if width > MAX_WIDTH:
        width = MAX_WIDTH
    if height > MAX_HEIGHT:
        height = MAX_HEIGHT
    
    frame =Frame(root, width=width, height=height)
    frame.grid(row=0,column=0)
    # Create canvas containing cell
    c = Canvas(frame, width=width, height=height, bg="white")
    # Create horizontal scroll bar
    hbar=Scrollbar(frame,orient=HORIZONTAL)
    hbar.pack(side=BOTTOM,fill=X)
    hbar.config(command=c.xview)
    # Create vertical scroll bar
    vbar=Scrollbar(frame,orient=VERTICAL)
    vbar.pack(side=RIGHT,fill=Y)
    vbar.config(command=c.yview)
    # Draw map here
    draw_board(c, graph, mCol, mRow)

    c.config(scrollregion=c.bbox("all"), xscrollcommand=hbar.set, yscrollcommand=vbar.set)
    c.pack(padx=10, pady=10)
    # Make program display at midle of the screen
    root.eval('tk::PlaceWindow %s center' % root.winfo_pathname(root.winfo_id()))


    root.update()

if __name__ == "__main__":
    main()

input('Press ENTER to exit')
