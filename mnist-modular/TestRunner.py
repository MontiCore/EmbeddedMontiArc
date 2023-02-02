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
        limit = sys.argv[4]

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
        #for c, e in commands, expected:
        while k < len(commands) and k < limit:
            commands[k].insert(0, execPath)
            print("Command: ", commands[k], " | Expected: ", expected[k])
            #process = subprocess.run(c, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, text=True, shell=True)
        #   output, error = process.communicate()
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

    numbers = []
    k = 0
    while k < 6:
        numbers.append(random.randint(0, 9))
        k += 1

    print("Numbers:", str(numbers))
    result = (numbers[2] + numbers[1] * 10 + numbers[0] * 100) + (numbers[5] + numbers[4] * 10 + numbers[3] * 100)
    print("Result: ", str(result))
    numbers.append(numbers)
    print(str(result))
    print(str(number_dirs))



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
