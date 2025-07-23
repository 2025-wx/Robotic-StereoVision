from scipy.spatial.transform import Rotation as R


quat = [-0.008474, 0.0077566, 0.70990852, -0.7042002]
r = R.from_quat(quat)
print(r.as_matrix())