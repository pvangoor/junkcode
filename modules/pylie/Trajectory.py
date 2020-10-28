import pylie

class Trajectory:
    def __init__(self, elements = None, times = None):
        if elements is None:
            self.elements = []
            self.times = []
        else:
            self.elements = elements

        if times is None:
            self.times = list(range(len(elements)))
        else:
            self.times = times
        

    def __getitem__(self, t):
        if isinstance(t, (int, float, complex)) and not isinstance(t, bool):
            # t is a number
            t = float(t)
            next_idx = [j for j in range(len(self.times)) if self.times[j] > t]
            if len(next_idx) == 0:
                next_idx = len(self.times)-1
            else:
                next_idx = next_idx[0]
            if next_idx == 0:
                next_idx = 1
            
            # Now (inter/extra)polate
            base_element = elements[next_idx-1]
            dt = self.times[next_idx] - self.times[next_idx-1]
            motion = (base_element.inv() * elements[next_idx]).log() / dt
            ndt = t - self.times[next_idx-1]
            elem = base_element * base_element.exp(ndt * motion)
            return elem

        raise NotImplementedError

    def begin(self):
        return self.times[0], self.elements[0]
    
    def end(self):
        return self.times[-1], self.elements[-1]

if __name__ == "__main__":
    import numpy as np
    times = [i * 0.1 for i in range(10)]
    motion = np.random.randn(3,1)
    elements = [pylie.SO3.exp(motion*t) for t in times]
    traj = Trajectory(elements, times)

    new_times = [0.2, 0.45, 1.2, -0.12]
    for t in new_times:
        group_error = traj[t] / pylie.SO3.exp(t*motion)
        norm_error = np.linalg.norm(group_error.as_matrix() - np.eye(3))
        print("The error at time {} is {}".format(t, norm_error))
        
