import mxnet as mx
import gluonnlp as nlp
ctx = mx.cpu()

lm_model, vocab = nlp.model.get_model(name='awd_lstm_lm_1150',
                                      dataset_name='wikitext-2',
                                      pretrained=True,
                                      ctx=ctx)

scorer = nlp.model.BeamSearchScorer(alpha=0, K=5)

# Transform the layout to NTC
def _transform_layout(data):
    if isinstance(data, list):
         return [_transform_layout(ele) for ele in data]
    elif isinstance(data, mx.nd.NDArray):
         return mx.nd.transpose(data, axes=(1, 0, 2))
    else:
         raise NotImplementedError

def decoder(inputs, states):
    states = _transform_layout(states)
    outputs, states = lm_model(mx.nd.expand_dims(inputs, axis=0), states)
    states = _transform_layout(states)
    return outputs[0], states

eos_id = vocab['.']
beam_size = 4
max_length = 20
sampler = nlp.model.BeamSearchSampler(beam_size=beam_size,
                                      decoder=decoder,
                                      eos_id=eos_id,
                                      scorer=scorer,
                                      max_length=max_length)

bos = 'I love it'.split()
bos_ids = [vocab[ele] for ele in bos]
begin_states = lm_model.begin_state(batch_size=1, ctx=ctx)
if len(bos_ids) > 1:
    _, begin_states = lm_model(mx.nd.expand_dims(mx.nd.array(bos_ids[:-1]), axis=1),
                               begin_states)
inputs = mx.nd.full(shape=(1,), ctx=ctx, val=bos_ids[-1])

# samples have shape (1, beam_size, length), scores have shape (1, beam_size)
samples, scores, valid_lengths = sampler(inputs, begin_states)

samples = samples[0].asnumpy()
scores = scores[0].asnumpy()
valid_lengths = valid_lengths[0].asnumpy()
print('Generation Result:')
for i in range(3):
    sentence = bos[:-1] + [vocab.idx_to_token[ele] for ele in samples[i][:valid_lengths[i]]]
    print([' '.join(sentence), scores[i]])

for beam_size in range(4, 17, 4):
    sampler = nlp.model.BeamSearchSampler(beam_size=beam_size,
                                          decoder=decoder,
                                          eos_id=eos_id,
                                          scorer=scorer,
                                          max_length=20)
    samples, scores, valid_lengths = sampler(inputs, begin_states)
    samples = samples[0].asnumpy()
    scores = scores[0].asnumpy()
    valid_lengths = valid_lengths[0].asnumpy()
    sentence = bos[:-1] + [vocab.idx_to_token[ele] for ele in samples[0][:valid_lengths[0]]]
    print([beam_size, ' '.join(sentence), scores[0]])