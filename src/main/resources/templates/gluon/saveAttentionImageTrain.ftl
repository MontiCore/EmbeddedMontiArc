<#-- (c) https://github.com/MontiCore/monticore -->
                    if save_attention_image == "True":
                        import matplotlib
                        matplotlib.use('Agg')
                        import matplotlib.pyplot as plt
                        logging.getLogger('matplotlib').setLevel(logging.ERROR)

                        if(os.path.isfile('src/test/resources/training_data/Show_attend_tell/dict.pkl')):
                            with open('src/test/resources/training_data/Show_attend_tell/dict.pkl', 'rb') as f:
                                dict = pickle.load(f)

                        plt.clf()
                        fig = plt.figure(figsize=(15,15))
                        max_length = len(labels)-1

                        ax = fig.add_subplot(max_length//3, max_length//4, 1)
                        ax.imshow(train_images[0+single_pu_batch_size*(batch_i)].transpose(1,2,0))

                        for l in range(max_length):
                            attention = attentionList[l]
                            attention = mx.nd.slice_axis(attention, axis=0, begin=0, end=1).squeeze()
                            attention_resized = np.resize(attention.asnumpy(), (8, 8))
                            ax = fig.add_subplot(max_length//3, max_length//4, l+2)
                            if int(labels[l+1][0].asscalar()) > len(dict):
                                ax.set_title("<unk>")
                            elif dict[int(labels[l+1][0].asscalar())] == "<end>":
                                ax.set_title(".")
                                img = ax.imshow(train_images[0+single_pu_batch_size*(batch_i)].transpose(1,2,0))
                                ax.imshow(attention_resized, cmap='gray', alpha=0.6, extent=img.get_extent())
                                break
                            else:
                                ax.set_title(dict[int(labels[l+1][0].asscalar())])
                            img = ax.imshow(train_images[0+single_pu_batch_size*(batch_i)].transpose(1,2,0))
                            ax.imshow(attention_resized, cmap='gray', alpha=0.6, extent=img.get_extent())

                        plt.tight_layout()
                        target_dir = 'target/attention_images'
                        if not os.path.exists(target_dir):
                            os.makedirs(target_dir)
                        plt.savefig(target_dir + '/attention_train.png')
                        plt.close()
