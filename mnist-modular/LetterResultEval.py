import sys

def runner():

    correctPath = None
    infResPath = None
    comPath = None

    if len(sys.argv) != 4:
        print(str("Not enough args. Try: python {} <commandsFile> <correctResultsFile> <inferenceResultsFile>").format(sys.argv[0]))
        exit(1)

    comPath = sys.argv[1]
    correctPath = sys.argv[2]
    infResPath = sys.argv[3]

    labelCoordinates = readCommandFile(comPath)
    correctResults = readLabelResults(correctPath)
    inferenceResults = readInferenceResults(infResPath)



    correct = 0
    wrong = 0

    k = 0
    while k < len(inferenceResults) and k < len(labelCoordinates):
        infResult = inferenceResults[k]
        coordinate = labelCoordinates[k]
        #print("CorrectRes Length: ", str(len(correctResults)))
        #print("Current Coordinate: ", str(coordinate))

        #print("LabelCoord Length: ", str(len(labelCoordinates)))
        #print("Inf Length: ", str(len(inferenceResults)))


        realResult = correctResults[coordinate]

        print("Round:", str(k), "| Real:", str(realResult), "| Found:", str(infResult))


        if infResult == realResult:
            correct += 1
        else:
            wrong += 1

        k += 1

    print("Correct Results:", str(correct), "| Wrong Results:", str(wrong))

def readLabelResults(path):
    correctResults = []
    f = open(path, "r")
    for line in f:
        line = line[:-1]
        
        numStr = line.split(".")[0]
        num = int(numStr)
        correctResults.append(num)

    f.close()

    return correctResults


def readInferenceResults(path):
    infResults = []
    f = open(path, "r")
    for line in f:
        line = line[:-1]
        if line == "" or "Correct" in line:
            continue

        num = str(line)
        infResults.append(num)

    f.close()

    return infResults

def readCommandFile(path):
    labelCoordinates = []
    readLine = False
    f = open(path, "r")
    for line in f:
        line = line[:-1]
        if line == "-|-|-":
            readLine = True
            continue

        if not readLine:
            continue

        dirArray = line.replace("[", "").replace("]", "").replace(" ", "").replace("'", "").split("/")
        picName = dirArray[len(dirArray)-1]
        picValueStr = picName.split(".")[0]
        picValue = int(picValueStr)

        labelCoordinates.append(picValue)

    f.close()

    return labelCoordinates

runner()