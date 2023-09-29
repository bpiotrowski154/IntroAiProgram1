import networkx as nx
import modules

done = False
graph = modules.createGraph()
cityList = modules.generateCityList()

#print(graph)

while not done:
    print("   City  List   ")
    print("----------------")
    count = 1
    for city in cityList:
        print(str(count) + '. ' + city)
        count += 1
    
    #User Validation
    valid = False
    while not valid:
        startCity = input("Enter the City to Start in: ")
        if startCity not in cityList:
            print("City not in Database\n")
            continue
        else:
            while not valid:
                endCity = input("Enter the City to go to: ")
                if endCity not in cityList:
                    print("City not in Database\n")
                    continue
                valid = True

    print("             Search  Options             ")
    print("-----------------------------------------")
    print("1. Brute Force Search")
    print("2. Breadth-First Search")
    print("3. Depth-First Search")
    print("4. Iterative Deepening Depth-First Search")
    print("5. Best-First Search")
    print("6. A* Search")
    print("0. Perform All Searches")

    #user input validation
    valid = False
    while not valid:
        try:
            searchChoice = int(input("Input Number of Desired Search Method: "))
            if searchChoice > -1 and searchChoice < 7:
                valid = True
            else:
                print("Invalid Input\n")

        except:
            print("Invalid Input\n")
    
    print("")
    match searchChoice:
        case 1:
            modules.bruteForceSearch(graph, startCity, endCity)
        case 2:
            modules.breadthFirstSearch(graph, startCity, endCity)
        case 3:
            modules.depthFirstSearch(graph, startCity, endCity)
        case 4:
            modules.iterativeDeepeningDepthFirstSearch(graph, startCity, endCity)
        case 5:
            modules.bestFirstSearch(graph, startCity, endCity)
        case 6:
            modules.aStarSearch(graph, startCity, endCity)
        case 0:
            modules.bruteForceSearch(graph, startCity, endCity)
            modules.breadthFirstSearch(graph, startCity, endCity)
            modules.depthFirstSearch(graph, startCity, endCity)
            modules.iterativeDeepeningDepthFirstSearch(graph, startCity, endCity)
            modules.bestFirstSearch(graph, startCity, endCity)
            modules.aStarSearch(graph, startCity, endCity)
        case _:
            print("Unexpected Error cannot proceed with search")
    
    #user input validation
    valid = False
    while not valid:
        proceed = input("Would you like to do another (y/n): ")
        if proceed.capitalize() == "Y":
            valid = True
        elif proceed.capitalize() == "N":
            valid = True
            done = True
        else:
            print("Invalid Input\n")