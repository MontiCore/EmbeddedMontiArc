import h5py
import mxnet
from mxnet import gluon, autograd

# Path to your .h5 file
file_path = 'train.h5'

# Open the HDF5 file
with h5py.File(file_path, 'r') as h5_file:
    # Assume the dataset is stored under a group name 'images'
    print(h5_file.keys())
    dataset = h5_file['data'][:]
    labelset = h5_file['softmax_label'][:]


inputData = mxnet.nd.array(dataset, ctx = mxnet.cpu())
labelData = mxnet.nd.array(labelset)
labelData = labelData.astype('int')

print(labelData)
batch_size = 1

true_input = mxnet.nd.ones((int(50000/batch_size),batch_size,1,28,28))
true_label = mxnet.nd.ones((int(50000/batch_size),batch_size))

for i in range(int(50000/batch_size)):
    true_input[i] = inputData[batch_size*i:batch_size*i+batch_size]
    true_label[i] = labelData[batch_size*i:batch_size*i+batch_size]

possibleLabels = list(range(16))

net = gluon.nn.Sequential()

with net.name_scope():
    net.add(
        gluon.nn.Conv2D(channels=20, kernel_size=3, strides=2),  # First convolutional layer
        gluon.nn.Activation("relu"),  # ReLU activation function
        gluon.nn.AvgPool2D(pool_size=2),  # Average pooling layer
        gluon.nn.Conv2D(channels=50, kernel_size=3, strides=2),  # Second convolutional layer
        gluon.nn.Activation("relu"),  # ReLU activation function
        gluon.nn.AvgPool2D(pool_size=2),  # Second average pooling layer
        gluon.nn.Flatten(),  # Flatten the output for Dense layers
        gluon.nn.Dense(500),  # Fully connected layer with 500 units
        gluon.nn.Activation("relu"),  # ReLU activation function
        gluon.nn.Dropout(0.2),  # Dropout layer with 20% dropout rate
        gluon.nn.Dense(16)  # Final fully connected layer with 16 units
         # Softmax activation function
    )

# Initialize parameters
net.initialize(init=mxnet.init.Xavier())  # Using Xavier initialization

mxnet.gluon.data.vision.transforms.ToTensor()

epochs = 2
lossFunc = gluon.loss.SoftmaxCrossEntropyLoss()
trainer = gluon.Trainer(net.collect_params(),'sgd',{'learning_rate' : 0.01,'wd': 0.0001})



for i in range(epochs):
    loss = 0.
    index = 0
    for data in true_input:
        with autograd.record():
            output = net(data)
            print(data.shape)
            loss = lossFunc(output, (true_label[index]))
        loss.backward()
        print(loss)
        trainer.step(batch_size)
        index += 1


print("TRIALS BEGIN")
print("---------------------------")

file_path = 'test.h5'

# Open the HDF5 file
with h5py.File(file_path, 'r') as h5_file:
    # Assume the dataset is stored under a group name 'images'
    print(h5_file.keys())
    dataset = h5_file['data'][:]
    labelset = h5_file['softmax_label'][:]
    true_input = mxnet.nd.ones((int(50000/batch_size),batch_size,1,28,28))
    true_label = mxnet.nd.ones((int(50000/batch_size),batch_size))

    for i in range(int(10000/batch_size)):
        true_input[i] = inputData[batch_size*i:batch_size*i+batch_size]
        true_label[i] = labelData[batch_size*i:batch_size*i+batch_size]

for i in range(10):
    print(f'Label for sample {i}: {true_label[i]}')

    # Model inference
    trial = net(mxnet.nd.array(true_input[i]))  # Convert to MXNet NDArray if needed
    prediction = (mxnet.nd.softmax(trial).asnumpy().flatten()).argmax()  # Apply softmax if necessary

    # Print prediction
    print(f'Prediction for sample {i}: {prediction}')

correct_predictions = 0
total_samples = len(true_input)  # Assuming true_input contains all 10,000 samples

for i in range(total_samples):
    # Model inference
    trial = net(mxnet.nd.array(true_input[i]))  # Convert to MXNet NDArray if needed
    prediction = (mxnet.nd.softmax(trial).asnumpy().flatten()).argmax()  # Apply softmax and get the predicted class

    # Check if the prediction matches the true label
    if prediction == true_label[i]:
        correct_predictions += 1

# Compute accuracy
accuracy = correct_predictions / total_samples
print(f'Accuracy: {accuracy * 100:.2f}%')












print("Goodbye World")


