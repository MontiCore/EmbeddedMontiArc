import mxnet as mx
import warnings
import sys

from mxnet import gluon

ctx = mx.gpu() if mx.context.num_gpus() else mx.cpu()
inputName = None
modelPath = None
paramsPath = None
network = None
oldNetwork = None
input_shape = None

oldNetworkPath = None

newModelDirectory = None
newModelName = None
parameterLayers = None
reExport = False

index = 0
while index < len(sys.argv):
    arg = sys.argv[index]

    if arg == "-in":
        inputName = sys.argv[index + 1]
    elif arg == "-mp":
        modelPath = sys.argv[index + 1]
    elif arg == "-pp":
        paramsPath = sys.argv[index + 1]
    elif arg == "-nmd":
        newModelDirectory = sys.argv[index + 1]
    elif arg == "-nmn":
        newModelName = sys.argv[index + 1]
    elif arg == "-pl":
        paramList = sys.argv[index + 1]
        parameterLayers = paramList.split(",")
    elif arg == "-onp":
        oldNetworkPath = sys.argv[index + 1]
    elif arg == "-re":
        reExport = True
    elif arg == "-shape":
        input_shape_str = sys.argv[index + 1]
        input_shape_map = {}
        shape_pairs = input_shape_str.split(";")
        for pair in shape_pairs:
            key, value_str = pair.split(":")
            values = [int(dim) for dim in value_str.split(",")]
            input_shape_map[key] = values

    index += 1

if reExport:
    #warnings.simplefilter("ignore")
    print("Old network: ", oldNetworkPath)
    print("Old params: ", paramsPath)
    print("Input name: ", inputName)
    oldNetwork = gluon.nn.SymbolBlock.imports(oldNetworkPath, [inputName], paramsPath, ctx=ctx)
    #oldNetwork.load_parameters(paramsPath)
    oldNetwork.initialize()
    print("re-exporting parameters of old network to:", paramsPath)
    oldNetwork.collect_params()
    oldNetwork.export(oldNetworkPath)
    oldNetwork.save_parameters(paramsPath, deduplicate=True)

"""
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    oldNetwork = gluon.nn.SymbolBlock.imports(oldNetworkPath, [inputName], paramsPath, ctx=ctx, allow_missing=True)
"""

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    network = gluon.nn.SymbolBlock.imports(modelPath, [inputName], paramsPath, ctx=ctx, ignore_extra=True)

if network is not None :
    file_save_dir = newModelDirectory + "/" + newModelName
    assert len(input_shape_map) == 1 , f"Single input shape expected, got {len(input_shape_map)}"
    input_shape_list = list(input_shape_map.values())[0]
    print(input_shape_list)
    input_shape_ndarray = mx.nd.zeros((1, *tuple(input_shape_list)))
    network.forward(input_shape_ndarray)
    network.export(file_save_dir)


