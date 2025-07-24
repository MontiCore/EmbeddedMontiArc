# (c) https://github.com/MontiCore/monticore

#Arg1 "cpu" or "gpu"

# '0':
python3 TestMNIST.py --image resources/test_image_0.png --digit 0 --ctx ${1:-"cpu"}

# '1':
python3 TestMNIST.py --image resources/test_image_1.png --digit 1 --ctx ${1:-"cpu"}

# '2': Will be predicted as 8
#python3 TestMNIST.py --image resources/test_image_2.png --digit 2 --ctx ${1:-"cpu"}

# '3':
python3 TestMNIST.py --image resources/test_image_3.png --digit 3 --ctx ${1:-"cpu"}

# '4':
python3 TestMNIST.py --image resources/test_image_4.png --digit 4 --ctx ${1:-"cpu"}

# '5':
python3 TestMNIST.py --image resources/test_image_5.png --digit 5 --ctx ${1:-"cpu"}

# '6':
python3 TestMNIST.py --image resources/test_image_6.png --digit 6 --ctx ${1:-"cpu"}

# '7':
python3 TestMNIST.py --image resources/test_image_7.png --digit 7 --ctx ${1:-"cpu"}

# '8':
python3 TestMNIST.py --image resources/test_image_8.png --digit 8 --ctx ${1:-"cpu"}

# '9': Predicted as 4
#python3 TestMNIST.py --image resources/test_image_9.png --digit 9 --ctx ${1:-"cpu"}