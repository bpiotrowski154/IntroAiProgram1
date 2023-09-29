import csv
import geopy.distance
import time
import heapq

def createGraph():
    graph = {}

    #create coordinates of cities
    with open("IntroAiProgram1\coordinates.csv", "r") as file:
        csv_reader = csv.reader(file)
        coordList = []
        for row in csv_reader:
            coordList.append(row)
    file.close()

    #create graph
    with open("IntroAiProgram1\Adjacencies.txt", "r") as file1:
        r = csv.reader(file1, delimiter=' ')
        for line in r:
            adjacencyA = line[0]
            adjacencyB = line[1]
            
            for list in coordList:
                if list[0] == adjacencyA:
                    adjacencyACoords = [float(list[1]), float(list[2])]
                    continue
                elif list[0] == adjacencyB:
                    adjacencyBCoords = [float(list[1]), float(list[2])]
                    continue
            
            distanceBetweenAdjacencies = geopy.distance.geodesic(adjacencyACoords, adjacencyBCoords).miles

            #if key is present in list, append value
            if adjacencyA in graph.keys():
                if adjacencyB not in (x[0] for x in graph[adjacencyA]):
                    graph[adjacencyA].append([adjacencyB,distanceBetweenAdjacencies])
            else:
                graph[adjacencyA] = []
                graph[adjacencyA].append([adjacencyB,distanceBetweenAdjacencies])
            
            if adjacencyB in graph.keys():
                if adjacencyA not in (x[0] for x in graph[adjacencyB]):
                    graph[adjacencyB].append([adjacencyA,distanceBetweenAdjacencies])
            else:
                graph[adjacencyB] = []
                graph[adjacencyB].append([adjacencyA,distanceBetweenAdjacencies])
    file1.close()
    return graph


def generateCityList():
    cityList = []

    with open("IntroAiProgram1\coordinates.csv", "r") as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            cityList.append(row[0])
    file.close()
    return cityList

#uses breadth-first search to traverse the entire graph of cities
#and stores each node in a list, then searches through the list for the ending city
#Then it checks the adjacencys between each of the cities between start and end for validity 
#and sets the path as the elements in between the two.
def bruteForceSearch(graph, start, goal):
    startTime = time.time()
    time.sleep(1)
    queue = [(start, 0, None)]
    visited = []
    

    while queue:
        (x, distance, parentNode) = queue.pop(0)
        if x not in (x[0] for x in visited):
            visited.append((x, distance, parentNode))
            for neighbor in graph[x]:
                queue.append((neighbor[0], neighbor[1], x))

    if goal not in (x[0] for x in visited):
        print("Brute Force Search path not found")
        endTime = time.time()
        print("Time Taken to Search: " + str(endTime - startTime) + " seconds")
        print('\n')
        return
    
    index = [x for x,y  in enumerate(visited) if y[0] == goal]
    (currentNode, distance, parentNode) = visited.pop(index[0])
    correct= [(currentNode, distance, parentNode)]

    while start not in (x[0] for x in correct):
        index = [y for y, z in enumerate(visited) if z[0] == parentNode]
        (currentNode,distance, parentNode) = visited.pop(index[0])
        correct.insert(0, (currentNode, distance, parentNode))

    (currentNode, distance, parentNode) = correct.pop(0)
    pathAsString = currentNode
    totDistance = distance

    for item in correct:
        pathAsString += ' -> ' + item[0]
        totDistance += item[1]

    print("Brute Force Search path exists: " + pathAsString)
    print("Total Distance: " + str(round(totDistance,2)) + " miles")
    endTime = time.time()
    print("Time Taken to Search: " + str(endTime - startTime) + " seconds")
    print('\n')
    return


def breadthFirstSearch(graph, start, goal):
    startTime = time.time()
    time.sleep(1)
    distance = 0
    queue = [(start, distance, [start])]
    visited = []

    while queue:
        (x, curDistance, path) = queue.pop(0)
        if x not in visited:
            if x == goal:
                pathAsString = path.pop(0)
                for element in path:
                    pathAsString += ' -> ' + element

                print("Breadth-First Search path Exists: " + pathAsString)
                print("Total Distance: " + str(round(curDistance,2)) + " miles")
                end = time.time()
                print("Time Taken to Search: " + str(end-startTime) + " seconds")
                print('\n')
                return
            visited.append(x)
            for neighbor in graph[x]:
                queue.append((neighbor[0], curDistance + neighbor[1], path + [neighbor[0]]))

    print("Breadth-First Search path not found")
    end = time.time()
    print("Time Taken to Search: " + str(end-startTime) + " seconds")
    print('\n')
    return


#This dfs algorithm that returns a path, whil slightly modified,
#was obtained from the user XueYu on the stackoverflow page: 
# https://stackoverflow.com/questions/12864004/tracing-and-returning-a-path-in-depth-first-search
def depthFirstSearch(graph, start, goal):
    startTime = time.time()
    time.sleep(1)
    distance = 0
    stack = [(start, distance, [start])] #second value in tuple is for the path
    visited = []

    while stack:
        (x, curDistance, path) = stack.pop()
        if x not in visited:
            if x == goal:
                pathAsString = path.pop(0)
                for element in path:
                    pathAsString += ' -> ' + element

                print("Depth-First Search path Exists: " + pathAsString)
                print("Total Distance: " + str(round(curDistance,2)) + " miles")
                end = time.time()
                print("Time Taken to Search: " + str(end-startTime) + " seconds")
                print('\n')
                return
            visited.append(x)
            for neighbor in graph[x]:
                stack.append((neighbor[0], curDistance + neighbor[1], path + [neighbor[0]]))

    print ("Depth-First Search path not found")
    end = time.time()
    print("Time Taken to Search: " + str(end-startTime) + " seconds")
    print('\n')
    return

#Pseudocode for functions iterativeDeepenignDepthFirstSearch and depthLimitedSearch were obtained
#from chatGPT using the prompt: "Can you generate pseudocode for iterative deepening depth first search on
#an undirected graph" -> "Can you modify it so that it returns the path found"
def iterativeDeepeningDepthFirstSearch(graph, start, goal):
    startTime = time.time()
    time.sleep(1)
    depthLimit = 0
    found = False
    path = []

    while not found:
        visited = {}
        found, path = depthLimitedSearch(graph, start, goal, depthLimit, visited, path)
        depthLimit += 1

    if found:
        currentNode = path.pop(0)
        pathAsString = currentNode
        totDistance = 0
        for node in path:
            pathAsString += ' -> ' + node
            for neighbor in graph[currentNode]:
                if neighbor[0] == node:
                    totDistance += neighbor[1]
                    break
            currentNode = node

        print("Iterative Deepening Depth-First Search path exists: " + pathAsString)
        print("Total Distance: " + str(round(totDistance,2)) + " miles")
        endTime = time.time()
        print("Time Taken to Search: " + str(endTime-startTime) + " seconds")
        print('\n')
        return
    else:
        print("Iterative Deepening Depth-First Search path not found")
        endTime = time.time()
        print("Time Taken to Search: " + str(endTime-startTime) + " seconds")
        print('\n')
        return

def depthLimitedSearch(graph, node, target, depthLimit, visited, path):
    if node == target:
        return True, path+[node]
    if depthLimit == 0:
        return False, path
    
    visited[node] = True

    for neighbor in graph[node]:
        if neighbor[0] not in visited:
            found, newPath = depthLimitedSearch(graph, neighbor[0], target, depthLimit - 1, visited, path+[node])
            if found:
                return True, newPath
    
    return False, path

#Algorithm created based on pseudocode found on pg. 87 of textbook
def bestFirstSearch(graph, start, goal):  
    startTime = time.time()
    time.sleep(1)
    queue = [(0, start, None)]
    visited = []

    #While queue is not empty
    while queue:
        #remove minimum value from queue
        (distance, x, parentNode) = queue.pop(0)
        #If the current node doesn't exist in the visited list continue
        if x not in (x[0] for x in visited):
            visited.append([x, parentNode, distance])
            if x == goal:
                correct = []
                (currentNode, parentNode, distance) = visited.pop()
                correct.insert(0, (currentNode, parentNode, distance))
                
                while start not in (x[0] for x in correct):
                    #insert parent node
                    index = [y for y, z in enumerate(visited) if z[0] == parentNode]
                    (currentNode,parentNode, distance) = visited.pop(index[0])
                    correct.insert(0, (currentNode, parentNode, distance))

                (currentNode, parentNode, distance) = correct.pop(0)
                pathAsString = currentNode
                totDistance = distance

                for item in correct:
                    pathAsString += ' -> ' + item[0]
                    totDistance += item[2]

                print("Best-First Search path exists: " + pathAsString)
                print("Total Distance: " + str(round(totDistance,2)) + " miles")
                endTime = time.time()
                print("Time Taken to Search: " + str(endTime - startTime) + " seconds")
                print('\n')
                return
            
            #While x has children
            childList = graph[x].copy()
            while childList:
                i = 0
                #if the child is not in the queue then add it
                if childList[i][0] not in (x[1] for x in queue):
                    queue.append((childList[i][1], childList[i][0], x))
                    childList.pop(i)
                else:
                    #if the the child is already in the queue then delete it
                    childList.pop(i)
                i += 1
            #Sort the queue in ascending order of distance
            queue.sort()

    print ("Best-First Search path not found")
    endTime = time.time()
    print("Time Taken to Search: " + str(endTime - startTime) + " seconds")
    print('\n')
    return

# Original algorithm implementation for A* and heuristic acquired from: 
# https://saturncloud.io/blog/implementing-the-a-algorithm-in-python-a-stepbystep-guide/
# Algorithm has been modified to fit current data structures
def aStarSearch(graph, start, goal):
    startTime = time.time()
    time.sleep(1)
    open = []
    closed = []
    startCost = 0

    heapq.heappush(open, (startCost, start, None))

    while open:
        currentCost, currentNode, currentNodeParent = heapq.heappop(open)

        if currentNode == goal:
            path = []
            path.insert(0, (currentNode, currentNodeParent))
            parentNode = currentNodeParent
            while start not in (x[0] for x in path):
                    #insert parent node
                    index = [y for y, z in enumerate(closed) if z[0] == parentNode]
                    (pathNode,parentNode) = closed.pop(index[0])
                    path.insert(0, (pathNode, parentNode))

            (tempPath, throwAwayParent) = path.pop(0) 
            pathAsString = tempPath
            totDistance = 0

            for node in path:
                pathAsString += ' -> ' + node[0]
                for neighbor in graph[tempPath]:
                    if neighbor[0] == node[0]:
                        totDistance += neighbor[1]
                        break
                tempPath = node[0]

            print("A* path exists: " + pathAsString)
            print("Total Distance: " + str(round(totDistance,2)) + " miles")
            endTime = time.time()
            print("Time Taken to Search: " + str(endTime - startTime) + " seconds")
            print('\n')
            return

        closed.append((currentNode, currentNodeParent))

        for neighbor in graph[currentNode]:
            neighborParent = currentNode
            neighborCost =  currentCost + 1

            if neighbor[0] in closed:
                continue

            newCost = currentCost + 1
            if neighbor[0] not in open:
                heapq.heappush(open, (newCost + heuristic(neighbor[0], goal), neighbor[0], neighborParent))
            elif newCost < neighborCost:
                neighborCost = newCost
    print("A* path not found")   
    endTime = time.time()
    print("Time Taken to Search: " + str(endTime - startTime) + " seconds")    
    print('\n')
    return

def heuristic(node, goal):
    x1, y1, x2, y2 = None, None, None, None
    with open("IntroAiProgram1\coordinates.csv", "r") as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if row[0] == node:
                x1 = float(row[1])
                y1 = float(row[2])
            if row[0] == goal:
                x2 = float(row[1])
                y2 = float(row[2])
            if x1 != None and y1 != None and x2 != None and y2 != None:
                break    
    file.close()
    return abs(x1 - x2) + abs(y1 - y2)