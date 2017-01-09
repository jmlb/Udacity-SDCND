import math

from pprint import pprint



def batches(batch_size, features, labels):
    """
    Create batches of features and labels
    :param batch_size: The batch size
    :param features: List of features
    :param labels: List of labels
    :return: Batches of (Features, Labels)
    """
    assert len(features) == len(labels)
    batch_split = []
    for i, example in enumerate(features):
        batch_split.append([example, labels[i]])
        
        
        
# 4 Samples of features
features = [
    ['F11','F12','F13','F14'],
    ['F21','F22','F23','F24'],
    ['F31','F32','F33','F34'],
    ['F41','F42','F43','F44']]
# 4 Samples of labels
labels = [
    ['L11','L12'],
    ['L21','L22'],
    ['L31','L32'],
    ['L41','L42']]

    
outout_batches = []
sample_size = len(features)
for start_i in range(0, sample_size, batch_size):
   end_i = start_i + batch_size
   batch = [features[start_i:end_i], labels[start_i:end_i]]
   outout_batches.append(batch)
        
print(outout_batches)