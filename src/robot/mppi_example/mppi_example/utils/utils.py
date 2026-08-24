import numpy as np
import yaml
from pathlib import Path
import scipy.stats as stats


def npprint_suppress():
    np.set_printoptions(suppress=True, precision=10)


def truncated_normal_sampler(mean, std, lower_bound, upper_bound, size=1):
    if std == 0:
        return np.ones(size) * mean
    a, b = (lower_bound - mean) / std, (upper_bound - mean) / std
    return stats.truncnorm.rvs(a, b, loc=mean, scale=std, size=size)    

def readTXT(filename):
    with open(filename) as f:
        lines = f.readlines()
    return lines


class ConfigYAML():
    """
    Config class for yaml file
    Able to load and save yaml file to and from python object
    """
    def __init__(self) -> None:
        pass
    
    def load_file(self, filename):
        d = yaml.safe_load(Path(filename).read_text())
        for key in d: 
            setattr(self, key, d[key]) 
    
    def save_file(self, filename):
        def np_convert(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, np.floating):
                return float(obj)
            elif isinstance(obj, np.integer):
                return int(obj)
            else:
                return obj
        
        d = vars(self)
        class_d = vars(self.__class__)
        d_out = {}
        for key in list(class_d.keys()):
            if not (key.startswith('__') or \
                    key.startswith('load_file') or \
                    key.startswith('save_file')):

                d_out[key] = np_convert(class_d[key])
        for key in list(d.keys()):
            if not (key.startswith('__') or \
                    key.startswith('load_file') or \
                    key.startswith('save_file')):
                d_out[key] = np_convert(d[key])
        with open(filename, 'w+') as ff:
            yaml.dump_all([d_out], ff)
