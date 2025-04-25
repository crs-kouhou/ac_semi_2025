import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# データ読み込み
n = int(input())
p = [0] * n
for i in range(n):
	p[i] = tuple(float(e) for e in input().split())
k = int(input())
ls = [0] * k
qs = [0] * k
for ik in range(k):
	m = int(input())
	l = [0] * m
	for im in range(m):
		l[im] = tuple(float(e) for e in input().split())
	ls[ik] = l
	nk = int(input())
	q = [0] * nk
	for ink in range(nk):
		q[ink] = tuple(float(e) for e in input().split())
	qs[ik] = q
p = np.array(p)
ls = np.array(ls)
qs = np.array(qs)

fig, ax = plt.subplots()

# アニメーション更新関数
def update(frame):
	ax.cla()

	ax.scatter(p[:, 0], p[:, 1], color='red', s=20)
	ax.scatter(qs[frame, :, 0], qs[frame, :, 1], color='blue', s=20)

	for li in ls[frame]:
		ax.plot (
			[li[0], li[1]],
			[li[2], li[3]], lw=2
		)
	return []

# アニメーション作成
frames = range(k)
ani = FuncAnimation(fig, update, frames=frames, blit=True)

# GIFに保存
ani.save("../data/lines_and_points.gif", writer=PillowWriter(fps=2))
