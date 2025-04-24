import numpy as np
import matplotlib.pyplot as plt

# 春ロボ2025のフィールド作り


def Reclist(x, y, sqw, sqh):
    return [
        [x, y, x + sqw, y],
        [x, y, x, y + sqh],
        [x + sqw, y, x + sqw, y + sqh],
        [x, y + sqh, x + sqw, y + sqh],
    ]


# (x,y,sqw,sqh)
wd = 38
reclist = [
    [0, 0, 3500 - 2 * wd, 7000 - 2 * wd],
    [0, 1462 + 1500, 550, wd],
    [550, 1462, wd, 4038],
    [550 + wd, 1462 + wd + 962, 1000, wd],
    [550 + wd, 1462 + 3 * (wd + 962), 1000, wd],
    [3500 - wd - wd - 897 - wd, 1462, wd, 4038],
    [550 + wd + 900, 1462, 1000, wd],
    [550 + wd + 900, 1462 + 2 * (962 + wd), 1000, wd],
    [550 + wd + 900, 1462 + 4 * (962 + wd), 1000, wd],
]

linesgroup = []

for i in range(len(reclist)):
    linesgroup.append(
        Reclist(reclist[i][0], reclist[i][1], reclist[i][2], reclist[i][3])
    )

# さらにここで追加を入れる。斜めの線など
# linesgroup.append([[1000,1000,3000,3000],
#                    [0,6000,3000,3000]])
# 結合
lines = [line for sublist in linesgroup for line in sublist]

np.save("../data/spring25", lines)
#


# 田巻のほしい形式
def tamaki():
    def to_polygon(rec: list[int]) -> str:
        ret = "4\n"
        ret += f"{rec[0]} {rec[1]}\n"
        ret += f"{rec[0]} {rec[1] + rec[3]}\n"
        ret += f"{rec[0] + rec[2]} {rec[1] + rec[3]}\n"
        ret += f"{rec[0] + rec[2]} {rec[1]}\n"
        return ret

    content = f"{len(reclist)}\n"
    for rec in reclist:
        content += to_polygon(rec)
    with open("../data/field.dat", "w") as f:
        f.write(content)


tamaki()


def ConvertPoints(LINES):
    POINTS = []
    n = 1 / 50
    for li in LINES:
        ld = np.sqrt((li[0] - li[2]) ** 2 + (li[1] - li[3]) ** 2)
        pl = np.array(
            [
                np.linspace(li[0], li[2], round(n * ld + 1)),
                np.linspace(li[1], li[3], round(n * ld + 1)),
            ]
        ).T
        for p in pl:
            POINTS.append(p)
    POINTS = np.array(POINTS).T
    return POINTS


fig = plt.figure(figsize=(6, 10))
plt.gca().set_aspect("equal", adjustable="box")  # アスペクト比を1に設定


p = ConvertPoints(lines)

plt.plot(p[0], p[1], "o", markersize=2)
plt.savefig("../data/field.jpeg")
# plt.show()
print(p.shape)
