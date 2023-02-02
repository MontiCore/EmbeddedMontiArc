import random
import subprocess
import sys, os


def runner():

    execPath = None
    scenario = None
    picture_set_path = None
    staticPath = None
    staticMode = False
    limit = 100


    if len(sys.argv) == 3:
        print("Running static mode")
        staticMode = True
        staticPath = sys.argv[1]
        scenario = sys.argv[2]
    elif len(sys.argv) <= 2:
        print(
            str("Not enough args. Example: \"python {} <execPath> <picturePath> <letters/digits> <limit?>\". Aborting.").format(
                sys.argv[0]))
        exit(1)
    elif len(sys.argv) >= 4:
        execPath = sys.argv[1]
        scenario = sys.argv[3]
        picture_set_path = sys.argv[2]





    if len(sys.argv) == 5:
        limit = int(sys.argv[4])



    if scenario != "digits" and scenario != "letters":
        print("Test scenario not digits or letters. Aborting. ->", scenario)
        exit(1)

    commands = []
    expected = []

    if staticMode:
        commands, expected = readListFromFile(staticPath)
        limit = len(commands)
    elif scenario == "digits":
        commands, expected = digitScenario(picture_set_path, limit)
    elif scenario == "letters":
        commands, expected = letterScenario(picture_set_path, limit)

    print("Limit is", str(limit))

    if not staticMode:
        for c in commands:
            c.insert(0, execPath)

        if len(commands) > 0 and len(expected) > 0 :
            writeCommandsToFile(commands, expected, scenario+"-static-run.txt")

    if commands is not None and len(commands) > 0:
        k = 0
        posCounter = 0
        negCounter = 0

        falseStorage = []
        results = []

        while k < len(commands) and k < limit:

            #print("Command: ", str(commands[k]), " | Expected: ", str(expected[k]) + "\n")
            process = subprocess.Popen(commands[k], text=True, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
            output, error = process.communicate()

            #if len(output) > 0:
            #    print("Output:", str(output))

            #if len(error) > 0:
            #    print("Error:", str(error))

            lines = str(output).split("\n")
            lastLine = lines[len(lines) - 2]
            # secondLastLine = lines[len(lines)-2]

            # print("Last:", lastLine)
            # print("SecondLast", secondLastLine)


            try:
                resNumStr = lastLine.replace(" ", "").split(":")[1]
                resNumStr = resNumStr.replace("\"", "")
                resNum = int(resNumStr)
                results.append(resNum)
                print("Result:", str(resNum), "| Expected:", str(expected[k]))

                if scenario == "digits":
                    if resNum == expected[k][6]:
                        posCounter += 1
                    else:
                        negCounter += 1
                        expected[k].append(resNum)
                        falseStorage.append(expected[k])
            except:
                print("result parsing exception")

            k += 1

        if scenario == "digits":
            print("Correct Results: ", str(posCounter), " | Wrong Results: ", str(negCounter))
            if negCounter > 0:
                print("Wrong results:\n")
                for f in falseStorage:
                    print(str(f))

                print("Correct Results: ", str(posCounter), " | Wrong Results: ", str(negCounter))

        print("Results:")
        for r in results:
            print(r)

        print("Correct Results: ", str(posCounter), " | Wrong Results: ", str(negCounter))
        print(scenario, "scenario done. Writing results")
        writeResultsToFile(scenario+"-results.txt", results)

    else:
        print("Commands are None. Error happened. Aborting.")
        exit(1)


def digitScenario(picture_path, limit):
    number_dirs = []
    k = 0
    while k < 10:
        number_dirs.append(picture_path + "/" + str(k) + "/")
        k += 1

    commands = []
    expected = []

    i = 0
    while i < limit:
        numbers = []
        k = 0
        while k < 6:
            numbers.append(random.randint(0, 9))
            k += 1

        com = []

        for n in numbers:
            numPath = picture_path + "/" + str(n) + "/"
            com.append(getRandomPictureForDigit(numPath))

        result = (numbers[2] + numbers[1] * 10 + numbers[0] * 100) + (numbers[5] + numbers[4] * 10 + numbers[3] * 100)
        numbers.append(result)

        expected.append(numbers)
        commands.append(com)

        # print("Com:", com, " | Exp:", expected)

        i += 1

    return commands, expected


def getRandomPictureForDigit(path):
    allPics = os.listdir(path)
    randPicIndex = random.randint(0, len(allPics) - 1)
    picName = allPics[randPicIndex]
    fullPath = path + picName
    return fullPath


def letterScenario(picture_path, limit):
    commands = []
    expect = []
    allFiles = os.listdir(picture_path)

    k = 0
    while k < len(allFiles) and k < limit:
        f = allFiles[k]
        path = picture_path + "/" + f
        commands.append([path])
        expect.append([path])
        k += 1

    return commands, expect

def writeCommandsToFile(commands, expected, path):
    clearFile(path)

    f = open(path, "w")
    for c in commands:
        f.write(str(c) + "\n")

    f.write("-|-|-\n")

    for e in expected:
        f.write(str(e) + "\n")

    f.close()

def readListFromFile(path):
    commands = []
    expected = []

    cDone = False

    f = open(path, "r")
    for line in f:
        line = line[:-1]
        #print(line)
        if line == "-|-|-":
            cDone = True
            continue

        comArr = line.replace("[", "").replace("]", "").replace(" ", "").replace("'", "").split(",")

        if cDone:
            expected.append(comArr)
        else:
            commands.append(comArr)

    f.close()



    if len(commands) == 0 or len(expected) == 0:
        print("Commands:", str(len(commands)), " | Expected: ", str(len(expected)))
        print("No commands or expected results loaded from file. Aborting")
        exit(1)

    return commands, expected

def writeResultsToFile(path,results):
    clearFile(path)

    f = open(path, "w")
    f.write("Corr")
    for c in results:
        f.write(str(c) + "\n")

    f.close()

def clearFile(path):
    try:
        os.remove(path)
    except:
        print("could not remove:", path)

runner()
