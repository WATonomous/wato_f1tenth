#import flax
import jax
from jax import random
import numpy as np
import jax.numpy as jnp
#from flax.training import orbax_utils
#import orbax
from functools import partial

def load_state(state, info, path=""):
    print(path)
    orbax_checkpointer = orbax.checkpoint.PyTreeCheckpointer()
    ckpt = {'state': state, 'info': info}
    load_filename = path
    state_restored = orbax_checkpointer.restore(load_filename, item=ckpt)
    print('Load from model: ', load_filename)
    return state_restored['state'], state_restored['info'].copy()

def numpify(x):
    return jax.device_get(x)

class Polar2Cartesian():
    def cartesian_to_polar(self, x, y):
        """Convert Cartesian coordinates to polar coordinates."""
        r = jnp.sqrt(x**2 + y**2)
        theta = jnp.arctan2(y, x)
        return r, theta

    def polar_to_cartesian(self, r, theta):
        """Convert polar coordinates to Cartesian coordinates."""
        x = r * jnp.cos(theta)
        y = r * jnp.sin(theta)
        return x, y
    

class oneLineJaxRNG:
    def __init__(self, init_num=0) -> None:
        self.rng = jax.random.PRNGKey(init_num)
    
    def new_key(self):
        self.rng, key = random.split(self.rng)
        return key
        
def generate_perms(rng_key, data_length, batch_size):
    # perms = jax.random.permutation(rng_key, data_length) # jax version is slow
    perms = np.random.permutation(data_length)
    steps_per_epoch = data_length//batch_size
    perms = perms[: steps_per_epoch * batch_size]  # skip incomplete batch
    perms = perms.reshape((steps_per_epoch, batch_size))
    return perms

def truncated_gaussian_sampler(rng_key, mean, lower_bound, upper_bound, shape, std=1):
    arr = jax.random.truncated_normal(key=rng_key, lower=(lower_bound-mean)/std, upper=(upper_bound-mean)/std, shape=shape)
    arr = arr * std + mean
    return arr

def cholesky_truncated_gaussian_2d_adjusted(rng_key, rng_key2, cov, lower_bounds, upper_bounds, N, mean):
    """
    Generate N samples from a 2D truncated Gaussian distribution using Cholesky decomposition
    with adjusted bounds for the truncated normals.
    
    Parameters:
    cov (numpy.ndarray): Covariance matrix
    lower_bounds (numpy.ndarray): Lower bounds for each dimension
    upper_bounds (numpy.ndarray): Upper bounds for each dimension
    N (int): Number of samples to generate
    mean (numpy.ndarray): Mean of the distribution (default [0, 0])
    
    Returns:
    numpy.ndarray: N samples from the 2D truncated Gaussian distribution
    """
    # Step 1: Cholesky decomposition of covariance matrix
    L = jnp.linalg.cholesky(cov)
    lower_z1, upper_z1 = lower_bounds[0] / L[0, 0], upper_bounds[0] / L[0, 0]
    lower_z2 = (lower_bounds[1] - L[1, 0] * lower_z1) / L[1, 1]
    upper_z2 = (upper_bounds[1] - L[1, 0] * upper_z1) / L[1, 1]
    
    # Step 2: Adjust the bounds for independent 1D truncated normal samples
    adjusted = jnp.array([[lower_z1, upper_z1], [lower_z2, upper_z2]])
    
    # Step 3: Sample N independent truncated normal samples with adjusted bounds
    z1 = truncated_gaussian_sampler(rng_key, mean[0], adjusted[0][0], adjusted[0][1], shape=N)
    z2 = truncated_gaussian_sampler(rng_key2, mean[1], adjusted[1][0], adjusted[1][1], shape=N)
    z = jnp.vstack((z1, z2)).T  # Stack the z1 and z2 samples together (N x 2 array)
    
    # Step 4: Apply the Cholesky transformation to introduce correlation
    correlated_samples = jnp.dot(z, L.T)
    
    return correlated_samples

class PositionalEncoding_jax():
    def __init__(self, L):
        self.L = L
        if L != 0:
            self.val_list = []
            for l in range(L):
                self.val_list.append(2.0 ** l)
            self.val_list = jnp.array(self.val_list)

    def encode(self, x):
        if self.L == 0:
            return x
        return jnp.sin(self.val_list * jnp.pi * x), jnp.cos(self.val_list * jnp.pi * x)

    def encode_even(self, x):
        if self.L == 0:
            return x
        return jnp.sin(self.val_list * jnp.pi * 2 * x), jnp.cos(self.val_list * jnp.pi * 2 * x)
    
    @partial(jax.jit, static_argnums=(0,2))
    def batch_encode(self, batch, loop_ind=1):
        if self.L == 0:
            return batch
        batch_encoded_list = []
        for ind in range(batch.shape[loop_ind]):
            encoded_ = self.encode(batch[:, ind, None])
            batch_encoded_list.append(encoded_[0])
            batch_encoded_list.append(encoded_[1])
        batch_encoded = jnp.stack(batch_encoded_list)
        batch_encoded = batch_encoded.transpose(1, 2, 0).reshape((batch_encoded.shape[1], 
                                                                  self.L * batch_encoded.shape[0]))
        return batch_encoded
    
    def decode(self, sin_value, cos_value):
        atan2_value = jnp.arctan2(sin_value, cos_value) / (jnp.pi)
        if jnp.isscalar(atan2_value) == 1:
            if atan2_value > 0:
                return atan2_value
            else:
                return 1 + atan2_value
        else:
            atan2_value[jnp.where(atan2_value < 0)] = atan2_value[jnp.where(atan2_value < 0)] + 1
            return atan2_value
        
    def decode_even(self, sin_value, cos_value):
        atan2_value = jnp.arctan2(sin_value, cos_value) / jnp.pi/2
        if jnp.isscalar(atan2_value) == 1:
            if atan2_value < 0:
                atan2_value = 1 + atan2_value
            if jnp.abs(atan2_value - 1) < 0.001:
                atan2_value = 0
        else:
            atan2_value[jnp.where(atan2_value < 0)] = atan2_value[jnp.where(atan2_value < 0)] + 1
            atan2_value[jnp.where(jnp.abs(atan2_value - 1) < 0.001)] = 0
        return atan2_value

    def batch_decode(self, sin_value, cos_value):
        atan2_value = jnp.arctan2(sin_value, cos_value) / (jnp.pi)
        sub_zero_inds = jnp.where(atan2_value < 0)
        atan2_value[sub_zero_inds] = atan2_value[sub_zero_inds] + 1
        return atan2_value
    
    # @partial(jax.jit, static_argnums=(0))
    def batch_decode2(self, list_data):
        dim = int(list_data.shape[1]/2)
        list_data = list_data.reshape(list_data.shape[0], dim, 2)
        atan2_value = jnp.arctan2(list_data[:, :, 0], list_data[:, :, 1]) / (np.pi)
        # sub_zero_inds = jnp.where(atan2_value < 0)
        atan2_value = jnp.where(atan2_value < 0, atan2_value + 1, atan2_value)
        # atan2_value = atan2_value.at[sub_zero_inds].set(atan2_value[sub_zero_inds] + 1)
        return atan2_value


def unstack(x, axis=0):
    """The opposite of stack()."""
    shape = x.shape
    return [jnp.squeeze(y, axis=axis) for y in \
            jnp.split(x, shape[axis], axis=axis)]
# jnp.unstack = unstack

