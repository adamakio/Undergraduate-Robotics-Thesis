import numpy as np

from autograd import numpy as anp
from autograd import jacobian

import tensorflow as tf

tf.keras.utils.disable_interactive_logging()

def load_model():
    return tf.keras.models.load_model('saved_models/neural_kinematics')

def ffpass_tf(q):
    """ The feedforward function of our neural net
    """    
    qr = q.reshape((1, q.size))
    return mlp.predict(qr)[0]


def is_jacobian_correct(jacobian_fn, ffpass_fn, n_inputs=7):
    """ Check of the Jacobian using numerical differentiation
    """
    q = np.random.random((n_inputs,))
    epsilon = 1e-5
    """ Check a few columns at random
    """
    for idx in np.random.choice(n_inputs, 7, replace=False):
        q2 = q.copy()
        q2[idx] += epsilon
        num_jacobian = (ffpass_fn(q2, w1, b1, w2, b2, w3, b3) - ffpass_fn(q, w1, b1, w2, b2, w3, b3)) / epsilon
        computed_jacobian = jacobian_fn(q, w1, b1, w2, b2, w3, b3)
        
        if not all(abs(computed_jacobian[:, idx] - num_jacobian) < 1e-3): 
            return False
    return True

def get_weights():
    mlp = load_model()
    w1, b1 = mlp.layers[0].get_weights()
    w2, b2 = mlp.layers[1].get_weights()
    w3, b3 = mlp.layers[2].get_weights()
    return [w1, b1, w2, b2, w3, b3]


def ffpass_anp(q, w1, b1, w2, b2, w3, b3):
    # Compute output
    a1 = anp.dot(q, w1) + b1   # affine
    a1 = anp.tanh(a1)    # tanh
    a2 = anp.dot(a1, w2) + b2  # affine
    a2 = anp.tanh(a2)    # tanh
    a3 = anp.dot(a2, w3) + b3  # affine
    out = a3
    return out

def jacobian_autograd(q, w1, b1, w2, b2, w3, b3):
    q = np.array(q)
    return jacobian(ffpass_anp)(q, w1, b1, w2, b2, w3, b3)


if __name__ == '__main__':
    mlp = load_model()
    [w1, b1, w2, b2, w3, b3] = get_weights()
    x = [-0.3, -0.2, -0.1 , 0.1, 0.2 ,0.3, 0.4]
    J = jacobian_autograd(x, w1, b1, w2, b2, w3, b3)