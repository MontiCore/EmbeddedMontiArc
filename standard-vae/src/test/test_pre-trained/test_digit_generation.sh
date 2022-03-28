# (c) https://github.com/MontiCore/monticore

#Arg1 & Arg2: Values for 2D-latent Code
#Arg3 Expected Class

# '0': 1 2, 2 3, 0.9 0.5
./build/cpp/TestMNIST 0.2 1 0

# '1': -1 -1, -1.5 -1.8
./build/cpp/TestMNIST -1 -1 1

# '2':
./build/cpp/TestMNIST -0.4 0.5 2

# '3':
./build/cpp/TestMNIST 0.15 0.15 3

# '4':
./build/cpp/TestMNIST 1 -1 4

# '5': Recognized as 8
#./build/cpp/TestMNIST 0.3 0.15 5

# '6': 0.4 0
./build/cpp/TestMNIST 0.5 -0.1 6

# '7': 0 -1,8, -0.1 -1
./build/cpp/TestMNIST -0.1 -1 7

# '8':
./build/cpp/TestMNIST -0.2 0.1 8

# '9': Recognized as 4
#./build/cpp/TestMNIST 0 -0.5 9