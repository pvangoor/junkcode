import pylie

class Trajectory:
    def __init__(self, elements = None, times = None):
        if elements is None:
            self._elements = []
            self._times = []
        else:
            self._elements = elements

        if times is None:
            self._times = list(range(len(elements)))
        else:
            self._times = times
    
    def __len__(self):
        assert len(self._elements) == len(self._times)
        return len(self._elements)

    def __getitem__(self, t):
        if isinstance(t, (int, float, complex)) and not isinstance(t, bool):
            # t is a number
            t = float(t)
            next_idx = [j for j in range(len(self._times)) if self._times[j] > t]
            if len(next_idx) == 0:
                next_idx = len(self._times)-1
            else:
                next_idx = next_idx[0]
            if next_idx == 0:
                next_idx = 1
            
            # Now (inter/extra)polate
            base_element = self._elements[next_idx-1]
            dt = self._times[next_idx] - self._times[next_idx-1]
            motion = (base_element.inv() * self._elements[next_idx]).log() / dt
            ndt = t - self._times[next_idx-1]
            elem = base_element * base_element.exp(ndt * motion)
            return elem

        raise NotImplementedError

    def begin(self):
        return self._times[0], self._elements[0]
    
    def end(self):
        return self._times[-1], self._elements[-1]
    
    def group_type(self):
        if len(self._elements) > 0:
            return type(self._elements[0])
        else:
            return None

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
        
