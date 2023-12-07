def init(swarmsize, x, y, theta, a_ids):
    import math
    import random
    
    for i in range(swarmsize):
        x[i] = random.uniform(-4, 4)
        y[i] = random.uniform(-4, 4)
        a_ids[i] = i
        theta[i] = random.uniform(-math.pi, math.pi)

    return x, y, theta, a_ids