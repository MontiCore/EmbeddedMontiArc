

deploy_netu
data
conv1__w
conv1__bconv1_ "Conv*

stride*
exhaustive_search *
order"NCHW*

kernel2 :CUDNNl
conv1_pool1_ "MaxPool*

kernel*

stride*
cudnn_exhaustive_search *
order"NCHW2 :CUDNNw
pool1_
conv2__w
conv2__bconv2_ "Conv*

stride*
exhaustive_search *
order"NCHW*

kernel2 :CUDNNl
conv2_pool2_ "MaxPool*

kernel*

stride*
cudnn_exhaustive_search *
order"NCHW2 :CUDNNe
pool2_
fc2__w
fc2__bfc2_ "FC*
	use_cudnn*
order"NCHW*
cudnn_exhaustive_search 2 M
fc2_fc2_ "Relu*
cudnn_exhaustive_search *
order"NCHW2 :CUDNNc
fc2_
fc3__w
fc3__bfc3_ "FC*
	use_cudnn*
order"NCHW*
cudnn_exhaustive_search 2 W
fc3_predictions "Softmax*
order"NCHW*
cudnn_exhaustive_search 2 :CUDNN:data:conv1__w:conv1__b:conv2__w:conv2__b:fc2__w:fc2__b:fc3__w:fc3__b