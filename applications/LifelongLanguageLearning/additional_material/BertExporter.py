import gluonnlp as nlp
import mxnet as mx
from mxnet import gluon
from mxnet import nd


class BertExporter:
    def __init__(self, model_type="small", dataset="book_corpus_wiki_en_uncased"):
        if model_type == "small":
            self.model_type = "bert_12_768_12"
        elif model_type == "large":
            self.model_type = "bert_24_1024_16"

        self.dataset = dataset

        self.loadModel()

    def transformModel(self):
        self.model = ExtendedBert(self.model)
        bert_exporter.model(mx.nd.zeros((1,128)), mx.nd.zeros((1,128)), mx.nd.zeros((1,1)))
        print("Model transformed!")

    def loadModel(self):
        self.model, self.vocab = nlp.model.get_model(self.model_type, dataset_name=self.dataset, pretrained=True, use_pooler=True, use_classifier=False, use_decoder=False)
        print("Model loaded!")

    def exportModel(self):
        self.model.export("/home/julian/Documents/BERT_exporter/classBertSmallUnPooled", epoch=0)
        print("Model exported!")


class ExtendedBert(gluon.HybridBlock):
     def __init__(self, model, **kwargs):
         super(ExtendedBert, self).__init__(**kwargs)
         with self.name_scope():
             self.model = model

     def hybrid_forward(self, F, words, segments, valid_len):
             valid_len = F.reshape(valid_len, shape=(-1,))
             embedding = self.model(words, segments, valid_len)

             embedding = embedding[1]

             return embedding

if __name__ == '__main__':
    bert_exporter = BertExporter()
    bert_exporter.transformModel()

    #tokenizer = nlp.data.BERTTokenizer(bert_exporter.vocab, lower=True)
    #transform = nlp.data.BERTSentenceTransform(tokenizer, max_seq_length=512, pair=False, pad=False)
    #sample = transform(['Hello world!'])
    #words, valid_len, segments = mx.nd.array([sample[0]]), mx.nd.array([sample[1]]), mx.nd.array([sample[2]])
    #res = bert_exporter.model(words, segments, valid_len)

    bert_exporter.exportModel()
