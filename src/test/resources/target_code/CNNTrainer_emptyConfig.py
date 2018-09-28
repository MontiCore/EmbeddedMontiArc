from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import cv2
import logging
import CNNCreator_emptyConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    emptyConfig = CNNCreator_emptyConfig.CNNCreator_emptyConfig()
    emptyConfig.train(
    )

    print '\n********************************************'
    print("Loading Deploy model")

    context = 'gpu'
    if context == 'cpu':
        device_opts = core.DeviceOption(caffe2_pb2.CPU, 0)
        print("CPU mode selected")
    elif context == 'gpu':
        device_opts = core.DeviceOption(caffe2_pb2.CUDA, 0)
        print("GPU mode selected")

    LeNet.load_net(LeNet.INIT_NET, LeNet.PREDICT_NET, device_opts=device_opts)

    img = cv2.imread("3.jpg")                                   # Load test image
    img = cv2.resize(img, (28,28))                              # Resize to 28x28
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY )                # Covert to grayscale
    img = img.reshape((1,1,28,28)).astype('float32')            # Reshape to (1,1,28,28)
    workspace.FeedBlob("data", img, device_option=device_opts)  # FeedBlob
    workspace.RunNet('deploy_net', num_iter=1)                  # Forward

    print("\nInput: {}".format(img.shape))
    pred = workspace.FetchBlob("predictions")
    print("Output: {}".format(pred))
    print("Output class: {}".format(np.argmax(pred)))
