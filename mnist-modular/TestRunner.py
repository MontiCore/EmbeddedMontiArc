import random
import subprocess
import sys, os


def runner():
    if len(sys.argv) <= 4:
        print(str("No enough args. Example: \"python {} <execPath> <picturePath> <letters/digits> <limit?>\". Aborting.").format( sys.argv[0]))
        exit(1)

    execPath = sys.argv[1]
    scenario = sys.argv[3]
    picture_set_path = sys.argv[2]
    limit = 100


    if len(sys.argv) == 5:
        limit = int(sys.argv[4])

    print("Limit is", str(limit))

    if scenario != "digits" and scenario != "letters":
        print("Test scenario not digits or letters. Aborting.")
        exit(1)

    commands = []
    expected = []

    if scenario == "digits":
        commands, expected = digitScenario(picture_set_path, limit)
    elif scenario == "letters":
        commands, expected = letterScenario(picture_set_path, limit)

    if commands is not None and len(commands) > 0:
        k = 0
        posCounter = 0
        negCounter = 0

        while k < len(commands) and k < limit:
            commands[k].insert(0, execPath)
            print("Command: ", str(commands[k]), " | Expected: ", str(expected[k]) + "\n")
            process = subprocess.Popen(["ls", "-al"], text=True,  stderr=subprocess.PIPE, stdout=subprocess.PIPE)
            output, error = process.communicate()

            if len(output) > 0:
                print("Output:", str(output))

            if len(error) > 0:
                print("Error:", str(error))

            lines = str(output).split("\n")
            lastLine = lines[len(lines)-1]
            secondLastLine = lines[len(lines)-2]

            print("Last:", lastLine)
            print("SecondLast", secondLastLine)

            if scenario == "digits":
                pass

            k += 1


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

        #print("Com:", com, " | Exp:", expected)

        i += 1


    return commands, expected


def getRandomPictureForDigit(path):
    allPics = os.listdir(path)
    randPicIndex = random.randint(0, len(allPics)-1)
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


runner()
