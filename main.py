from tkinter import *
from queue import Queue
import queue
import math
import argparse
import os.path
from os import path
import time
import random
from tkinter import messagebox

# Declare constant variables
INF = int(1e9)
PI = 3.14159265359
BORDER = -INF
WALL_BORDER = -4
INSIDE_POLYGON = -0.5
START = 1
ROAD = 2.5
END = 4
CATCH = 2
DISTANCE = 40
EMPTYPOINT = 0
COLOR = [['deep sky blue', 'sky blue'], ['steel blue', 'light steel blue'], ['gold4', 'gold2'],
['sea green', 'dark sea green'], ['dark orange', 'orange'], ['orange red', 'tomato'],
['hot pink', 'light pink'], ['dark violet', 'purple'], ['chocolate3', 'chocolate1']]
TIME_PER_UNIT = 0.1
SEARCH_DELAY_TIME = TIME_PER_UNIT / 1000

class vertices:
    def __init__(self, x, y, type):
        self.x = x
        self.y = y
        self.type = type
        self.listAdjacency = []
        self.dist = 1
        self.widget = -1

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
        return self.y * (mCol) + self.x
        
    def __eq__(self, obj):
        return self.x == obj.x and self.y == obj.y
    def __lt__(self, other):
        return self.dist <= other.dist

# Read input file
def ReadFile(fileName):
    output = []
    cusor = 0
    with open(fileName,'r') as f:
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
    return (startPoint, endPoint, listCatchPoint, listPolygons, mRow, mCol)

# Initialize map
def initGraph(mRow, mCol, listPolygons, startPoint, endPoint, listCatchPoint):
    graph = []
    for y in range(mRow):
        for x in range(mCol):
            v = vertices(x, y, EMPTYPOINT)
            # Check border
            if x < 1 or y < 1 or x == mCol or y == mRow:
                v.type = BORDER
            else:
                # Check if vertex is catch point
                for catch in listCatchPoint:
                    if v == catch:
                        v.type = catch.type
                # check if vertex is one of verttex of polygon
                for polygon in listPolygons:
                    for vPolygon in polygon:
                        if v == vPolygon:
                            v.type = vPolygon.type
                # start point
                if (v == startPoint):
                    v.type = startPoint.type
                # end point
                elif v == endPoint:
                    v.type = endPoint.type
                # adding all moveable vertices to the vertex
                v.addMovableCell(mCol, mRow)
            graph.append(v)

    for i in range(mCol + 1):
        v = vertices(i, mRow, BORDER)
        graph.append(v)
    for i in range(mRow + 1):
        v = vertices(mCol, i, BORDER)
        graph.append(v)
    return graph

# Initial widgets
def createWidgets(root, mCol, mRow):
    width = mCol * DISTANCE + 200
    height = mRow * DISTANCE + 200
    if width > root.winfo_screenwidth() + 80:
        width = root.winfo_screenwidth() + 80
    if height > root.winfo_screenheight() + 80:
        height = root.winfo_screenheight() + 80
    
    root.title('1612083_1612033_1612019')

    
    frame =Frame(root, relief=SUNKEN)
    frame.grid_rowconfigure(0, weight=1)
    frame.grid_columnconfigure(0, weight=1)
    # Create canvas containing cell
    # Create horizontal scroll bar
    hbar=Scrollbar(frame,orient=HORIZONTAL)
    hbar.grid(row=1, column=0, sticky=E+W)

    # Create vertical scroll bar
    vbar=Scrollbar(frame,orient=VERTICAL)
    vbar.grid(row=0, column=1, sticky=N + S)
    
    # Draw map here
    # draw_board(c, graph, mCol, mRow)
    c = Canvas(frame, width=width, height=height, bg="white", scrollregion=(0, 0, mCol * DISTANCE + 150, mRow * DISTANCE + 150), xscrollcommand=hbar.set, yscrollcommand=vbar.set)
    c.grid(row=0, column=0, sticky=N+S+E+W)

    hbar.config(command=c.xview)
    vbar.config(command=c.yview)

    frame.pack()
    return c

# Initial interface 
def initInterface(graph, canvas, col, row):
    for i in range(-1, col):
        if i < col:
            canvas.create_text((i + 1) * DISTANCE + DISTANCE * 1.5, DISTANCE / 2, text=i + 1)
    for j in range(-1, row):
        canvas.create_text(DISTANCE / 2, (j + 1) * DISTANCE + DISTANCE * 1.5, text=j + 1)
    i = 0
    for v in graph:
        coords = ((v.x + 1) * DISTANCE, (v.y + 1) * DISTANCE, (v.x + 1) * DISTANCE + DISTANCE, (v.y + 1) * DISTANCE + DISTANCE)
        widget = canvas.create_rectangle(coords, width=1, state='disabled')
        graph[i].widget = widget
        i += 1

# Draw map by tkinter
def draw_board(c, graph, mCol, mRow):
    for v in graph:
        widget = v.widget
        if v.type == EMPTYPOINT:
            continue
        elif (v.type == BORDER):
            c.itemconfig(widget, fill='gray')
        elif v.type == START:
            c.itemconfig(widget, fill='red')
        elif v.type == END:
            c.itemconfig(widget, fill='blue')
        elif v.type == CATCH:
            c.itemconfig(widget, fill='red4')    
        elif (v.type >= END + 1):
            index = (v.type - (END + 1)) % COLOR.__len__()
            c.itemconfig(widget, fill=COLOR[index][0])
        elif (v.type <= WALL_BORDER):
            index = abs(v.type - (WALL_BORDER - 1)) % COLOR.__len__()
            c.itemconfig(widget, fill=COLOR[index][1])
        elif (v.type == ROAD):
            c.itemconfig(widget, fill='medium orchid')
        elif (v.type == INSIDE_POLYGON):
            c.itemconfig(widget, fill='gray90')

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
    listInvalid = []
    for vertice in graph:
        lastest = vertices(mCol, vertice.y, 0)
        count = 0
        if vertice.x < xMax and vertice.x > xMin and vertice.y < yMax and vertice.y > yMin and vertice.type >= EMPTYPOINT and vertice.type <= END:
            j = 0
            while j < polygon.__len__():
                if vertice != polygon[j] and orientation(vertice, lastest, polygon[j]) == 0 and isInsideLine(vertice, lastest, polygon[j]):
                    #check same plane:
                    nextVer = polygon[(j + 1) % polygon.__len__()]
                    preVer = polygon[polygon.__len__() - 1]
                    if (j - 1 >= 0):
                        preVer = polygon[j - 1]
                    if (not samePlane(preVer, polygon[j], nextVer)):
                        count += 1
                j += 1
            i = 0
            while i < listEdge.__len__():
                for temp in listEdge[i]:
                    if orientation(vertice, lastest, graph[temp]) == 0 and isInsideLine(vertice, lastest, graph[temp]):
                        count += 1
                        break
                i += 1
        if count % 2 == 1:
            if vertice.type == EMPTYPOINT:
                vertice.type = INSIDE_POLYGON
            else:
                listInvalid.append(vertice)
    return listInvalid
            
# Check if the point 
def checker(listEgde, point):
    for i in listEgde:
        if point in i:
            return True
    return False

# Create walls
def createWalls(graph, listPolygons, mRow, mCol):
    result = []
    visited = [False for i in range(graph.__len__())]
    path = [False for i in range(graph.__len__())]
    count = 1
    listInvalid = []
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
                    if not visited[point] and not checker(listEgde, point) and (graph[point].type == EMPTYPOINT or graph[point].type == END + count):
                        visited[point] = True
                        q.put(point)
                        path[point] = u
                        if point == nextVertice.callPoint(mCol):
                            q.queue.clear()
            tempArr = findPath(startVertice.callPoint(mCol), nextVertice.callPoint(mCol), path, tempArr)
            listEgde.append(tempArr)
            i += 1
        #change graph
        for temp in listEgde:
            for j in temp:
                graph[j].type = WALL_BORDER - count
        listInvalid += isInsidePolygon(listEgde, graph, polygon, mCol, mRow)
        result.append(listEgde)
        count += 1
    return (result, listInvalid)

# Breath first search algorithm to find the shotest path
def BFS(s, f, graph, col, row, c):
    visited = [False for i in range(graph.__len__())]
    path = [-1 for i in range(graph.__len__())]
    for i in range(graph.__len__()):
        visited[i] = False
        path[i] = -1
    q = Queue()
    visited[s.callPoint(col)] = True
    q.put(s.callPoint(col))
    while not q.empty():
        u = q.get()       
        # cứ mỗi lần lấy đỉnh kề ra, chúng ta sẽ cho các đa giác di chuyển và vẽ lại bản đồ
        for x, y in graph[u].listAdjacency:
            point = y * col + x
            
            if graph[point].type <= END and graph[point].type >= EMPTYPOINT:
                if not visited[point]:
                    if graph[point].type == EMPTYPOINT:
                        c.itemconfig(graph[point].widget, fill='pale green')
                        c.update()
                        time.sleep(SEARCH_DELAY_TIME)
                    visited[point] = True
                    q.put(point)
                    path[point] = u
                    if graph[point].type == EMPTYPOINT:
                        c.itemconfig(graph[point].widget, fill='')
                        c.update()
                    if graph[point] == f:
                        break
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
            if path[f] not in mlist:  mlist.append(path[f])
            return findPath(s, path[f], path, mlist)

# Dijktra algorithm
def dijktra(s, f, graph, mCol, mRow, c):
    dist = [INF for i in range(graph.__len__() + 5)]
    path = [-1 for i in range(graph.__len__() + 5)]
    
    dist[s.callPoint(mCol)] = 0
    pq = queue.PriorityQueue()
    start = vertices(s.x, s.y, 0)
    start.dist = 0
    pq.put(start)
    while not pq.empty():
        vertex = pq.get()
        u = vertex.callPoint(mCol)
        w = vertex.dist
        for x, y in graph[u].listAdjacency:
            point = y * mCol + x
            if graph[point].type <= END and graph[point].type >= EMPTYPOINT:
                if w + graph[point].dist < dist[point]:
                    if graph[point].type == EMPTYPOINT:
                        c.itemconfig(graph[point].widget, fill='pale green')
                        c.update()
                        time.sleep(SEARCH_DELAY_TIME)
                    dist[point] = w + graph[point].dist
                    v = vertices(x, y, 0)
                    v.dist = dist[point]
                    pq.put(v)
                    path[point] = u
                    if graph[point].type == EMPTYPOINT:
                        c.itemconfig(graph[point].widget, fill='')
                        c.update()
    resultArr = []
    resultArr = findPath(s.callPoint(mCol), f.callPoint(mCol), path, resultArr)
    return resultArr

# A* algorithm --- start
class Node:
        # Mỗi node đều chứa node cha
    def __init__(self,parent= None,Position = None):
        self.parent = parent
        self.Position = Position
    def __eq__(self,other):
        return self.Position == other.Position

#Tim kiem heuristic (A sao)
def AStart(data,start,end,mrow,mcol, c):
    #Tạo openlist và closelist
    # openlist là phần tử cần duyệt lại
    # closelist là phần tử ko cần duyệt
    open_list =  []
    close_list = []
    start_node = Node(None,start)
    end_node = Node(None,end)

    # Bắt đầu tại start node
    # g là khoảng cách từ node hiên tại đến nốt bắt đầu
    # h là khoảng cách từ node đích đến node hiện tai ct
    # ct tính child.h = abs(child.Position.x - end_node.Position.x) + abs(child.Position.y - end_node.Position.y) 

    #f là tổng g +h
    start_node.g = start_node.h = start_node.f = 0
    open_list.append(start_node)

    # Duyệt cho đến khi tìm ra điểm đích
    while (len(open_list))>0:
        curent_node = open_list[0] 
        current_index = 0
        
        for index,item in enumerate(open_list):
            if item.f < curent_node.f:
                curent_node = item
                current_index=  index
        mPoint = curent_node.Position.y * mcol + curent_node.Position.x
        if data[mPoint].type == EMPTYPOINT:
            c.itemconfig(data[mPoint].widget, fill='pale green')
            c.update()
            time.sleep(SEARCH_DELAY_TIME)
        # Lấy current_node ra thêm vào close_list 
        # bắt đầu tìm kiếm node con của current
        open_list.pop(current_index)
        close_list.append(curent_node)

        if curent_node == end_node:
            result = []
            while curent_node is not None: 
                result.append(vertices(curent_node.Position.x,curent_node.Position.y,ROAD).callPoint(mcol))
                check = curent_node
                curent_node = curent_node.parent         
            return result[1:-1]
        #Tìm các node xung quanh current node 
        children = []
        # Các trường hợp có thể đi được
        PotisonAround = [(0,1),(0,-1),(-1,0),(1,0),(1,1),(1,-1),(-1,-1),(-1,1)]
        for plus_Position in PotisonAround:
            new_node = (curent_node.Position.x + plus_Position[0],curent_node.Position.y + plus_Position[1])
            #  node mới tràn
            #col == 22 row =18
            if new_node[0] > (mcol)  or new_node[0]<0 or new_node[1] > (mrow-1) or new_node[1]<0:
                continue
             #  node mới trùng node bị khóa
            a = new_node[1]*mcol + new_node[0]
            if data[a].type < EMPTYPOINT or data[a].type > END:  
                continue
            vertax = vertices(new_node[0],new_node[1],0)
            node  = Node(curent_node,vertax)
            children.append(node)
        #Duyệt lại danh sách node con trong closelist để đưa vào openlist
        # Nếu trùng closelist bỏ qua
        for child in children:
            check=2
            for close in close_list:
                if close == child:
                    check= check-1
                    break
            # Tao g,h,f
            if check==2:
                child.g = curent_node.g +1
               # child.h = ((child.Position.GetX() - end_node.Position.GetX()) * 2) + ((child.Position.GetY() - end_node.Position.GetY()) * 2)
                child.h = math.sqrt((child.Position.x - end_node.Position.x)**2 + abs(child.Position.y - end_node.Position.y)**2 ) 
                child.f = child.g + child.h
            # Kiểm tra đã tồn tại trong openlist chưa
                for open in open_list:
                    if open == child and child.g >= open.g:
                        check= check-1
                        break  
                if check ==2:              
                    open_list.append(child)
        if data[mPoint].type == EMPTYPOINT:
            c.itemconfig(data[mPoint].widget, fill='')
            c.update()
    return []
# A* algorithm --- end

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
def createRoad(choose, graph, listCatchPoint, startPoint, endPoint, mCol, mRow, c, listEdge, listPolygon, checkMovable, listInvalid):
    i = 0
    countDis = 0
    if listCatchPoint.__len__() == 0:
        countDis += FindPathWhilePlygonMove(startPoint, endPoint, graph, listPolygon, mCol, mRow, c, listEdge, choose, checkMovable)
        if countDis == -1:
            return countDis
    else:
        visited = [False for q in range(listCatchPoint.__len__() + 1)]
        start = startPoint
        while i <= listCatchPoint.__len__() + 1:
            index = getMinDistance(listCatchPoint, start, visited, mCol, mRow)
            end = endPoint
            if i < listCatchPoint.__len__() or index != -1:
                end = listCatchPoint[index]
            countDis += FindPathWhilePlygonMove(start, end, graph, listPolygon, mCol, mRow, c, listEdge, choose, checkMovable)
            if countDis == -1:
                return countDis
            i += 1
            visited[index] = True
            start = end
    return countDis

# Get random valid directtion   
def checkDirection(graph, polygon, mCol, mRow):
    direction = [
        [1, 0], [-1, 0], # right, left
        [0, 1], [0, -1], # up, down
    ]
    mDirection = [0, 0]
    xMax, yMax, xMin, yMin = getMax(polygon, mCol, mRow)
    # random direction
    random.shuffle(direction)
    for dir in direction:
        # to right
        if dir == [1, 0] and xMax + dir[0] < mCol:
            cusor = yMin
            while cusor <= yMax:
                point = cusor * mCol + (xMax + dir[0])
                if graph[point].type != EMPTYPOINT:
                    break
                cusor+=1
            if cusor == yMax + 1:
                return dir
        # to left
        elif dir == [-1, 0] and xMin + dir[0] > 0:
            cusor = yMin
            while cusor <= yMax:
                point = cusor * mCol + (xMin + dir[0])
                if graph[point].type != EMPTYPOINT:
                    break
                cusor += 1
            if cusor == yMax + 1:
                return dir
        # to up
        elif dir == [0, 1] and yMax + dir[0] < mRow:
            cusor = xMin
            while cusor <= xMax:
                point = (yMax + dir[1]) * mCol + cusor
                if graph[point].type != EMPTYPOINT:
                    break
                cusor += 1
            if cusor == xMax + 1:
                return dir
        # to down
        elif dir == [0, -1] and yMin + dir[0] > 0:
            cusor = xMin
            while cusor <= xMax:
                point = (yMin + dir[1]) * mCol + cusor
                if graph[point].type != EMPTYPOINT:
                    break
                cusor += 1
            if cusor == xMax + 1:
                return dir
    return mDirection

# Moving all polygon
def movingPolygon(graph, listPolygon, mCol, mRow, c, listEdge):
    counter = 1
    # Delete inside polygon
    listCheckInside = list(filter(lambda x: x.type == INSIDE_POLYGON, graph))
    for temp in listCheckInside:
        graph[temp.callPoint(mCol)].type = EMPTYPOINT
        c.itemconfig(graph[temp.callPoint(mCol)].widget, fill='')
    # Each polygon
    for polygon in listPolygon:
        whichPolygon = END + counter
        # Get random available direction
        direction = checkDirection(graph, polygon, mCol, mRow)

        if direction != [0, 0]:
            
            listVertices = list(filter(lambda x: x.type == whichPolygon or x.type == -whichPolygon, graph))
            if direction[0] != 0:
                reverse = direction[0] > 0
                listVertices.sort(key=lambda v: v.x, reverse=reverse)
            elif direction[1] != 0:
                reverse = direction[1] > 0
                listVertices.sort(key=lambda v: v.y, reverse=reverse)
            # Move all point of the polygon to the direction
            check = False
            for vertex in listVertices:
                point = (vertex.y + direction[1]) * mCol + vertex.x + direction[0]
                if graph[point].type != EMPTYPOINT and abs(graph[point].type) != whichPolygon:
                    check = True
            if not check:
                for vertex in listVertices:
                    point = (vertex.y + direction[1]) * mCol + vertex.x + direction[0]
                    if vertex.type != graph[point].type:
                        graph[point].type = vertex.type
                        c.itemconfig(vertex.widget, fill='')
                        graph[vertex.callPoint(mCol)].type = EMPTYPOINT
                for v in polygon:
                    v.x += direction[0]
                    v.y += direction[1]
                listPolygon[counter - 1] = polygon
                newlistEdge = []
                for edges in listEdge[counter - 1]:
                    temp = []
                    for edge in edges:
                        temp.append(edge + direction[1] * mCol + direction[0])
                    newlistEdge.append(temp)
                listEdge[counter - 1] = newlistEdge
                # Get new list poin inside polygon
        isInsidePolygon(listEdge[counter - 1], graph, polygon, mCol, mRow)
        counter += 1

# Fiding path while the polygons are moving
def FindPathWhilePlygonMove(s, f, graph, listPolygon, mCol, mRow, c, listEdge, choose, checkMovable):
    start = s
    count = 0
    while True:
        time.sleep(TIME_PER_UNIT)
        resultArr = []
        if choose == 2:
            resultArr = AStart(graph, start, f, mRow, mCol, c)
        elif choose == 3:
            resultArr = dijktra(start, f, graph, mCol, mRow, c)
        else:
            resultArr = BFS(start, f, graph, mCol, mRow, c)
        if (resultArr.__len__() != 0):
            if checkMovable != 'y':
                for a in resultArr:
                    graph[a].type = ROAD
                draw_board(c, graph, mCol, mRow)
                c.update()
                return resultArr.__len__()
            temp = resultArr[resultArr.__len__() - 1]
            graph[temp].type = ROAD
            start = graph[temp]
            count += 1
        if resultArr.__len__() == 0:
            if checkMovable != 'y':
                return -1
            for x, y in graph[f.callPoint(mCol)].listAdjacency:
                point = y * mCol + x
                if graph[point].type == ROAD:
                    return count
        if checkMovable == 'y':
            movingPolygon(graph, listPolygon, mCol, mRow, c, listEdge)
        draw_board(c, graph, mCol, mRow)
        c.update()
        if checkMovable != 'y':
            break

# main
def main():
    parser = argparse.ArgumentParser()

    # #-db DATABSE -u USERNAME -p PASSWORD -size 20
    parser.add_argument("-filename", "--file", help="read file ")
    args = parser.parse_args()
    if path.exists(args.file):
        print("[1]: Breath First Search")
        print("[2]: A Start")
        print("[3]: Dijktra")
        choose = input("Choose Algorithm: ")
        checkMovable = input("Movable Polygons(y/n):")
        # Read data from the input file
        startPoint, endPoint, listCatchPoint, listPolygons, mRow, mCol = ReadFile(args.file)
        

        # From this line, Those codes below will create an interface for user.
        # Create root view which is containing all views
        root = Tk()
        # Create widgets:
        c = createWidgets(root, mCol, mRow)
        
        # Initialize graph
        graph = initGraph(mRow, mCol, listPolygons, startPoint, endPoint, listCatchPoint)

        # Initialize interface
        initInterface(graph, c, mCol, mRow)

        # Create basic map and  Wall
        listEdge, listInvalid = createWalls(graph, listPolygons, mRow, mCol)
        draw_board(c, graph, mCol, mRow)
        c.update()
        check = True
        # Check if start or end inside polygon
        if startPoint in listInvalid or endPoint in listInvalid:
            check = False
        for v in listCatchPoint:
            if v in listInvalid:
                listCatchPoint.remove(v)
        
        if check:
            # #find path from start to end
            if not choose.isdigit():
                choose = 2
            number = createRoad(int(choose), graph, listCatchPoint, startPoint, endPoint, mCol, mRow, c, listEdge, listPolygons, checkMovable, listInvalid)
            mString = 'Distance Of The Path: ' + str(number)

            if number == -1:
                mString = 'There is no path to the end '
                messagebox.showinfo("Result", mString)
            
            c.create_text(DISTANCE * 2, (mRow + 1) * DISTANCE + DISTANCE * 2, text=mString)
            root.update()
        else:
            messagebox.showinfo("Result", 'There is no path to the end ')
        root.update()
    else:
        print(' File Not Existed')

if __name__ == "__main__":
    main()

input('Press ENTER to exit')
