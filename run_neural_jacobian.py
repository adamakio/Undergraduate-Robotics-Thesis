import numpy as np
import tensorflow as tf

tf.keras.utils.disable_interactive_logging()

def get_model():
    return tf.keras.models.load_model('saved_models/neural_jacobian')

def predict(model, x):
    """ The feedforward function of our neural net
    """    
    x = np.array(x)
    qr = x.reshape((1, x.size))
    return model.predict(qr)[0]

# x = np.random.rand(1, 14)
# y = predict(get_model(), x)
