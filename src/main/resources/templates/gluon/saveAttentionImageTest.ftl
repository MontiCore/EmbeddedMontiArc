                    if save_attention_image == "True":
                        plt.clf()
                        fig = plt.figure(figsize=(15,15))
                        max_length = len(labels)-1


                        ax = fig.add_subplot(max_length//3, max_length//4, 1)
                        ax.imshow(test_images[0+test_batch_size*(batch_i)].transpose(1,2,0))

                        for l in range(max_length):
                            attention = attentionList[l]
                            attention = mx.nd.slice_axis(attention, axis=0, begin=0, end=1)
                            attention = mx.nd.squeeze(attention)
                            attention_resized = np.resize(attention.asnumpy(), (8, 8))
                            ax = fig.add_subplot(max_length//3, max_length//4, l+2)
                            if dict[int(mx.nd.slice_axis(mx.nd.argmax(outputs[l+1], axis=1), axis=0, begin=0, end=1).asscalar())] == "<end>":
                                ax.set_title(".")
                                img = ax.imshow(test_images[0+test_batch_size*(batch_i)].transpose(1,2,0))
                                ax.imshow(attention_resized, cmap='gray', alpha=0.6, extent=img.get_extent())
                                break
                            else:
                                ax.set_title(dict[int(mx.nd.slice_axis(mx.nd.argmax(outputs[l+1], axis=1), axis=0, begin=0, end=1).asscalar())])
                                img = ax.imshow(test_images[0+test_batch_size*(batch_i)].transpose(1,2,0))
                                ax.imshow(attention_resized, cmap='gray', alpha=0.6, extent=img.get_extent())


                        plt.tight_layout()
                        plt.savefig(target_dir + '/attention_test.png')
                        plt.close()