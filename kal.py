from math import *

# The Implementation of a 1D Kalman Filter
def measure (mu, va, x):
    reg = (1/sqrt(2*pi*va))
    est = exp(((-1/2)*(x-mu)**2)/va)
    return reg*est


def update (mu1, va1, mu2, va2):
    new_mean = ((va2*mu1) + (va1*mu2))/(va1+va2)
    new_var = (1/((1/va1)+(1/va2)))
    return new_mean, new_var
    
def predict(mu1, va1, mu2, va2):
    new_mean = mu1 + mu2
    new_var = va1 + va2
    return new_mean, new_var 



def main():
    measurements = [5., 6., 7., 9., 10.]
    motion = [1., 1., 2., 1., 1.]
    measurement_sig = 2.
    motion_sig = 2.
    mu = 0.
    sig = 0.00000001

    for i in range(len(measurements)):
        mu, sig = update(mu, sig, measurements[i], measurement_sig)
        print(f'update: {mu,sig}')
        mu, sig = predict(mu, sig, motion[i], motion_sig)
        print(f'predict: {mu, sig}')

main()