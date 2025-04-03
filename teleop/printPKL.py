import pickle

# 打开 pkl 文件（假设文件名为 'franka_traj.pkl'）
with open('franka_traj.pkl', 'rb') as file:
    data = pickle.load(file)

# 打印加载后的数据
print(data)