vector<float> applyBeamSearch(vector<float> input, int depth, int width, int maxDepth, double currProb, int netIndex, vector<float> bestOutput)
{
double bestProb = 0.0;
while (depth < maxDepth){
    depth ++;
    int batchIndex = 0;
    for batchEntry in input:
        top_k_indices = mx.nd.topk(batchEntry, axis=0, k=width);
        top_k_values = mx.nd.topk(batchEntry, ret_typ="value", axis=0, k=width);
        for index in range(top_k_indices.size):

            /*print mx.nd.array(top_k_indices[index]) */
            /*print top_k_values[index] */
            if depth == 1:
                /*print mx.nd.array(top_k_indices[index]) */
                result = applyBeamSearch(self._networks[netIndex](mx.nd.array(top_k_indices[index])), depth, width, maxDepth,
                    currProb * top_k_values[index], netIndex, self._networks[netIndex](mx.nd.array(top_k_indices[index])));
            else:
                result = applyBeamSearch(self._networks[netIndex](mx.nd.array(top_k_indices[index])), depth, width, maxDepth,
                    currProb * top_k_values[index], netIndex, bestOutput);

            if depth == maxDepth:
                /*print currProb */
                if currProb > bestProb:
                    bestProb = currProb;
                    bestOutput[batchIndex] = result[batchIndex];
                    /*print "new bestOutput: ", bestOutput */

        batchIndex ++;
    }
/*print bestOutput; */
/*cout << bestProb; */
return bestOutput;
}