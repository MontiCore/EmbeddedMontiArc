# !/usr/share/python3

import gluonnlp as nlp
import numpy as np
import mxnet as mx
import sys
import csv
import h5py
import time as t
import warnings


class EpisodicMemoryPreprocess:
    def __init__(self, data_path, file_names, train_sizes, test_sizes, dataset="book_corpus_wiki_en_uncased"):
        self.model_type = "bert_12_768_12"
        self.dataset = dataset
        self.data_path = data_path
        self.file_names = file_names[1:-1].split(",")

        self.total_samples = 0
        self.num_samples_train = []
        temp = train_sizes[1:-1].split(",")
        for s in temp:
            self.num_samples_train.append(int(s))
            self.total_samples += self.num_samples_train[-1]

        self.num_samples_test = []
        temp = test_sizes[1:-1].split(",")
        for s in temp:
            self.num_samples_test.append(int(s))
            self.total_samples += self.num_samples_test[-1]

        self.max_seq_len = 128
        self.data = np.array([])
        self.labels = np.array([])
        self.val_lens = np.array([])
        self.segs = np.array([])


    def process(self):
        start = t.time()

        self.model, self.vocab = nlp.model.get_model(self.model_type, dataset_name=self.dataset, pretrained=True,
                                                     use_pooler=False, use_classifier=False, use_decoder=False)

        tokenizer = nlp.data.BERTTokenizer(self.vocab, lower=True)
        transform = nlp.data.BERTSentenceTransform(tokenizer, max_seq_length=self.max_seq_len, pair=False, pad=True)
        total_ind = 0

        with open(self.data_path + "test/" + self.file_names[0], 'r') as file:

            test = h5py.File(self.data_path + "/test/test_" + str(0) + ".h5", "w")
            dset_data_0_test = test.create_dataset('data_0', shape=(self.num_samples_test[0], self.max_seq_len))
            dset_data_1_test = test.create_dataset('data_1', shape=(self.num_samples_test[0], self.max_seq_len))
            dset_data_2_test = test.create_dataset('data_2', shape=(self.num_samples_test[0], 1))
            dset_target_test = test.create_dataset('softmax_label', shape=(self.num_samples_test[0],))

            reader = csv.reader(file, delimiter=',')

            last_start_ind = 0

            for j, row in enumerate(reader):
                if j >= 1:

                    total_ind += 1

                    sample = transform((row[0],))
                    words, valid_len, segments = mx.nd.array([sample[0]]), mx.nd.array([sample[1]]), mx.nd.array([sample[2]])
                    valid_len = mx.nd.reshape(valid_len, shape=(-1,1))

                    if not len(self.data):
                        self.data = words
                        self.segs = segments
                        self.val_lens = valid_len
                        self.labels = [int(row[1])]
                    else:
                        self.data = mx.nd.concat(self.data, words, dim=0)
                        self.segs = mx.nd.concat(self.segs, segments, dim=0)
                        self.val_lens = mx.nd.concat(self.val_lens, valid_len, dim=0)
                        self.labels.append(int(row[1]))

                    if (total_ind + 1) % (self.total_samples / 1000) == 0:
                        print(str((total_ind + 1) / (self.total_samples / 100)) + "%")

                    if j % 2000 == 0:
                        len_data = len(self.data)
                        dset_data_0_test[last_start_ind:last_start_ind+len_data] = self.data.asnumpy()
                        dset_data_1_test[last_start_ind:last_start_ind+len_data] = self.segs.asnumpy()
                        dset_data_2_test[last_start_ind:last_start_ind+len_data] = self.val_lens.asnumpy()
                        dset_target_test[last_start_ind:last_start_ind+len_data] = np.array(self.labels)

                        last_start_ind = last_start_ind + len_data

                        self.data = np.array([])
                        self.segs = np.array([])
                        self.val_lens = np.array([])
                        self.labels = np.array([])

        if len(self.data):
            dset_data_0_test[last_start_ind:] = self.data.asnumpy()
            dset_data_1_test[last_start_ind:] = self.segs.asnumpy()
            dset_data_2_test[last_start_ind:] = self.val_lens.asnumpy()
            dset_target_test[last_start_ind:] = np.array(self.labels)

        test.close()

        print("Finished generating test data.")

        self.data = np.array([])
        self.segs = np.array([])
        self.val_lens = np.array([])
        self.labels = np.array([])

        for i, file in enumerate(self.file_names):

            with open(self.data_path + "train/" + file, 'r') as file:

                train = h5py.File(self.data_path + "/train/train_" + str(i) + ".h5", "w")
                dset_data_0_train = train.create_dataset('data_0', shape=(self.num_samples_train[0], self.max_seq_len))
                dset_data_1_train = train.create_dataset('data_1', shape=(self.num_samples_train[0], self.max_seq_len))
                dset_data_2_train = train.create_dataset('data_2', shape=(self.num_samples_train[0], 1))
                dset_target_train = train.create_dataset('softmax_label', shape=(self.num_samples_train[i],))

                last_start_ind = 0

                reader = csv.reader(file, delimiter=',')
                max_len = 0
                for j, row in enumerate(reader):
                    if j >= 1:
                        total_ind += 1

                        sample = transform((row[0],))
                        words, valid_len, segments = mx.nd.array([sample[0]]), mx.nd.array([sample[1]]), mx.nd.array([sample[2]])
                        valid_len = mx.nd.reshape(valid_len, shape=(-1,1))

                        if not len(self.data):
                            self.data = words
                            self.segs = segments
                            self.val_lens = valid_len
                            self.labels = [int(row[1])]
                        else:
                            self.data = mx.nd.concat(self.data, words, dim=0)
                            self.segs = mx.nd.concat(self.segs, segments, dim=0)
                            self.val_lens = mx.nd.concat(self.val_lens, valid_len, dim=0)
                            self.labels.append(int(row[1]))

                        if (total_ind+1) % (self.total_samples/1000) == 0:
                            print(str((total_ind+1)/(self.total_samples/100)) + "%")

                        if j % 2000 == 0:
                            len_data = len(self.data)
                            dset_data_0_train[last_start_ind:last_start_ind + len_data] = self.data.asnumpy()
                            dset_data_1_train[last_start_ind:last_start_ind + len_data] = self.segs.asnumpy()
                            dset_data_2_train[last_start_ind:last_start_ind + len_data] = self.val_lens.asnumpy()
                            dset_target_train[last_start_ind:last_start_ind + len_data] = np.array(self.labels)

                            last_start_ind = last_start_ind + len_data

                            self.data = np.array([])
                            self.segs = np.array([])
                            self.val_lens = np.array([])
                            self.labels = np.array([])

            if len(self.data):
                dset_data_0_train[last_start_ind:] = self.data.asnumpy()
                dset_data_1_train[last_start_ind:] = self.segs.asnumpy()
                dset_data_2_train[last_start_ind:] = self.val_lens.asnumpy()
                dset_target_train[last_start_ind:] = np.array(self.labels)
                
            train.close()

            print("Finished generating train dataset " + str(i) + ".")

            self.data = np.array([])
            self.segs = np.array([])
            self.val_lens = np.array([])
            self.labels = np.array([])

        end = t.time()
        print("Data Processed!" + " Elapsed Time: " + str(end-start) + "s")

if __name__ == '__main__':
    episodic_memory_preprocess = EpisodicMemoryPreprocess(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    episodic_memory_preprocess.process()
