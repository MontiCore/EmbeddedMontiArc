vector<float> applyBeamSearch(vector<float> input, int depth, int width, int maxDepth, double currProb, int netIndex, vector<float> bestOutput)
{
double bestProb = 0.0;
while (depth < maxDepth){
    depth ++;
    int batchIndex = 0;
    for batchEntry in input:
        int top_k_indices[width];
        float top_k_values[width];

        for(int i = 0; i < width; i++){
            vector<float> batchEntryCopy = batchEntry;
            std::nth_element(batchEntry.begin(), batchEntry.begin() + i, batchEntry.end());
            top_k_values[i] = batchEntry[i]
            std::vector<int>::iterator itr = std::find(batchEntryCopy.begin(), batchEntryCopy.end(), top_k_values[i]);
            top_k_indices[i] = std::distance(batchEntryCopy.begin(), itr);
        }

        for(int index = 0; index < width; index++){

            /*print mx.nd.array(top_k_indices[index]) */
            /*print top_k_values[index] */
            if (depth == 1){
                /*print mx.nd.array(top_k_indices[index]) */
                result = applyBeamSearch(self._networks[netIndex](mx.nd.array(top_k_indices[index])), depth, width, maxDepth,
                                    currProb * top_k_values[index], netIndex, self._networks[netIndex](mx.nd.array(top_k_indices[index])));
                _predictor_3_.predict(target_0_, decoder_state_, target_1_, decoder_state_, decoder_output_);

                result = applyBeamSearch(self._networks[netIndex](mx.nd.array(top_k_indices[index])), depth, width, maxDepth,
                    currProb * top_k_values[index], netIndex, self._networks[netIndex](mx.nd.array(top_k_indices[index])));
            }else{
                result = applyBeamSearch(self._networks[netIndex](mx.nd.array(top_k_indices[index])), depth, width, maxDepth,
                    currProb * top_k_values[index], netIndex, bestOutput);
            }

            if (depth == maxDepth){
                /*print currProb */
                if (currProb > bestProb){
                    bestProb = currProb;
                    bestOutput[batchIndex] = result[batchIndex];
                    /*print "new bestOutput: ", bestOutput */
                    }
            }

        batchIndex ++;
        }
    }
/*print bestOutput; */
/*cout << bestProb; */
return bestOutput;
}