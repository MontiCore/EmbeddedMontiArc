# (c) https://github.com/MontiCore/monticore
#C++ test:     ./build/cpp/TestMNIST resources/test_image_0.png 0
#Arg1 image path
#Arg2 digit class

# Python test: python3 TestMNIST.py --image resources/test_image_1.png --digit 1 --ctx "cpu"

# '0':
python3 TestMNIST.py --image resources/test_image_0.png --digit 0 --ctx $1

# '1':
python3 TestMNIST.py --image resources/test_image_1.png --digit 1 --ctx $1

# '2':
python3 TestMNIST.py --image resources/test_image_2.png --digit 2 --ctx $1

# '3':
python3 TestMNIST.py --image resources/test_image_3.png --digit 3 --ctx $1

# '4':
python3 TestMNIST.py --image resources/test_image_4.png --digit 4 --ctx $1

# '5':
python3 TestMNIST.py --image resources/test_image_5.png --digit 5 --ctx $1

# '6':
python3 TestMNIST.py --image resources/test_image_6.png --digit 6 --ctx $1

# '7':
python3 TestMNIST.py --image resources/test_image_7.png --digit 7 --ctx $1

# '8':
python3 TestMNIST.py --image resources/test_image_8.png --digit 8 --ctx $1

# '9':
python3 TestMNIST.py --image resources/test_image_9.png --digit 9 --ctx $1