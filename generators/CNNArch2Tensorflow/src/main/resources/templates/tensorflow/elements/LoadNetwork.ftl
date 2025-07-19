<#-- (c) https://github.com/MontiCore/monticore -->
<#assign networkDir = element.networkDir>
<#assign networkPrefix = element.networkPrefix>
<#assign numInputs = element.numInputs?c>
<#assign outputShape = tc.join(element.outputShape, ",")>
<#assign trainable = element.trainable>
            lastEpoch = 0
            hdf5File = None
            for file in os.listdir("${networkDir}"):
                if "${networkPrefix}" in file and ".hdf5" in file:
                    hdf5File = file

            if hdf5File:
                tmpmodel = tf.keras.models.load_model("${networkDir}/" + hdf5File, custom_objects={'tf': tf})
                # Add reversed permutation due to another permute layer in loaded model
                if "tf_hwc_permute" in [l.name for l in tmpmodel.layers]:
                    data_ = tf.keras.layers.Permute((3,1,2))(data_)
                ${element.name} = tf.keras.Sequential()
                for l in tmpmodel.layers:
<#if trainbable == false>
                    l.trainable = False
</#if>
                    ${element.name}.add(l)
                ${element.name} = ${element.name}(${tc.join(element.inputs, ",")})
            else:
                raise FileNotFoundError("Model file was not found in '${networkDir}'.")
