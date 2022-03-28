# (c) https://github.com/MontiCore/monticore

#Arg1 & Arg2: Values for 2D-latent Code
#Arg3 Expected Class

# '0':
./build/cpp/TestMNIST 1 0 0

# '1':
./build/cpp/TestMNIST -1 -1 1

# '2':
./build/cpp/TestMNIST 0.15 0.15 2

# '3':
./build/cpp/TestMNIST 0 1 3

# '4':
#No code found yet

# '5': recognized as 3
#./build/cpp/TestMNIST 0.2 0.45 5

# '6':
./build/cpp/TestMNIST 0.3 0 6

# '7': 0 -1,8, -0.1 -1
./build/cpp/TestMNIST -0.3 -0.25 7

# '8':
./build/cpp/TestMNIST 0.15 -0.6 8

# '9': 0.7 -0.9
./build/cpp/TestMNIST -0.4 0 9