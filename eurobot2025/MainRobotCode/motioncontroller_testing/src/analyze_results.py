import matplotlib.pyplot as plt
import csv
import numpy as np

def load_csv(filename):
    with open(filename, "r") as f:
        result = csv.DictReader(f)
        data = {k : [] for k in result.fieldnames}
        for l in result:
            for j in data:
                data[j].append(float(l[j]))
    return data

test_result = load_csv("test_result.csv")
ref = load_csv("reference_result.csv")

diff = {k: np.array(test_result[k]) - np.array(ref[k]) for k in test_result}

print(f"{'Average':20s}|{'New':10s}|{'Ref':10s}|{'Diff':10s}")
print(f"{'-------':20s}|{'---':10s}|{'---':10s}|{'---':10s}")
for k in test_result:
    print(f"{k:20s}|{np.average(test_result[k]):10.5f}|{np.average(ref[k]):10.5f}|{np.average(diff[k]):10.5f}")

n = len(test_result.keys()) // 3 + 1
fig, axs = plt.subplots(n, 3, sharex=True)
axs = axs.flatten()
X = 5 * np.array(range(len(diff[k])))
for k, a in zip(diff, axs):
    a.set_title(k)
    # a.bar(X, diff[k])
    # a.bar(X + 1, test_result[k])
    # a.bar(X + 2, ref[k])
    a.scatter(range(len(diff[k])), test_result[k])
    a.scatter(range(len(diff[k])), ref[k])

for a in axs:
    a.grid()

keys = ["AStarComputeDuration", "PathLength"]
fig, axs = plt.subplots(2, len(keys), sharex=True)
axs = axs.flatten()
for k, a in zip(keys, axs):
    a.set_title(k)
    a.scatter(range(len(diff[k])), np.array(test_result[k]) - np.array(ref[k]))

for k, a in zip(keys, axs[len(keys):]):
    a.set_title(k + ", relative")
    e = np.array(test_result[k]) - np.array(ref[k])
    a.scatter(range(len(diff[k])), e / np.array(ref[k]))

for a in axs:
    a.grid()

plt.show()