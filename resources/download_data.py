# (c) https://github.com/MontiCore/monticore  
import requests

urls = [
    'https://nlp.stanford.edu/projects/nmt/data/iwslt15.en-vi/train.en',
    'https://nlp.stanford.edu/projects/nmt/data/iwslt15.en-vi/train.vi',
    'https://nlp.stanford.edu/projects/nmt/data/iwslt15.en-vi/tst2013.en',
    'https://nlp.stanford.edu/projects/nmt/data/iwslt15.en-vi/tst2013.vi'
]

for url in urls:
    filename = url.split('/')[-1]

    with requests.get(url, stream=True) as r:
        r.raise_for_status()

        with open(filename, 'wb') as f:
            for chunk in r.iter_content(chunk_size=8192): 
                if chunk:
                    f.write(chunk)

    print(filename + ' downloaded')
