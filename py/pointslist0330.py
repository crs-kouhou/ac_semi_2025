import pygame
import sys
import numpy as np

# 初期化
pygame.init()

font = pygame.font.SysFont("Arial", 24)
# サイズ
width, height = 1000, 1000
screen = pygame.display.set_mode((width, height))

pygame.display.set_caption("gragh")
clock = pygame.time.Clock()
# color
white = (255, 255, 255)
black = (0, 0, 0)

# パラメータ
ox = round(width / 3)
oy = round(height * 4 / 5)


def O(x, y):  # 画面でのピクセルとしての座標
    return pygame.draw.circle(screen, black, (x, y), 5)


# 拡大率
a = 1 / 10
b = 1 / 10
s = 0


def X():
    return np.linspace(-ox / a, (width - ox) / a, width)


def Y():
    return np.linspace(oy / b, (height + oy) / b, height)


# 点を打つ
def P(x, y, c):
    return pygame.draw.circle(screen, c, (ox + a * x, oy - b * y), 3)


def Pos(x, y):  # ピクセルを座標にする
    return ((x - ox) / a, (-y + oy) / b)


# x,yのリスト表示
def Ps(x, y, c):
    for i in range(len(x)):
        pygame.draw.circle(screen, c, (ox + a * x[i], oy - b * y[i]), 2)


def Fn(x, y, c):
    for i in range(len(x)):
        if i > 0:
            pygame.draw.line(
                screen,
                c,
                (ox + a * x[i - 1], oy - b * y[i - 1]),
                (ox + a * x[i], oy - b * y[i]),
                2,
            )


def Line(x_1, y_1, x_2, y_2, c):
    pygame.draw.line(
        screen, c, (ox + a * x_1, oy - b * y_1, ox + a * x_2, oy - b * y_2), 2
    )


field = np.load("../data/spring25.npy")


def Lines(x, c):  # xは線分のリスト
    for i in range(len(x)):
        pygame.draw.line(
            screen,
            c,
            (ox + a * x[i][0], oy - b * x[i][1]),
            (ox + a * x[i][2], oy - b * x[i][3]),
            2,
        )


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


click_positions = []
start_ticks = pygame.time.get_ticks()

load_ticks = start_ticks
running = True
dragging = False
LEFT = False
RIGHT = False
UP = False
DOWN = False

while running:
    screen.fill(white)  # まずは白塗り
    game_time = pygame.time.get_ticks() - start_ticks
    # print(game_time)
    elapsed_time = pygame.time.get_ticks() - load_ticks  # 更新時間
    if elapsed_time > 1000:  # 1秒経ったら更新
        load_ticks = pygame.time.get_ticks()

    # pygame.draw.circle(screen,black,  (round(width/3),(round(height*2/3)))  ,3)
    P(0, 0, black)

    # LINES
    Lines(field, black)

    mouse_x, mouse_y = pygame.mouse.get_pos()

    mpos = Pos(mouse_x, mouse_y)
    mouse_pos_text = font.render(
        f"mouse: ({round(mpos[0])},{round(mpos[1])})", True, black
    )
    screen.blit(mouse_pos_text, (10, 10))  # 画面の左上に表示

    #
    for I in click_positions:
        P(I[0], I[1], black)

    # for posi in click_positions:
    #     pygame.draw.circle(screen, black, posi, 5)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # やめるイベント
            running = False

        # キーボード
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                UP = True
            elif event.key == pygame.K_DOWN:
                DOWN = True
            elif event.key == pygame.K_RIGHT:
                RIGHT = True
            elif event.key == pygame.K_LEFT:
                LEFT = True

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                UP = False
            elif event.key == pygame.K_DOWN:
                DOWN = False
            elif event.key == pygame.K_RIGHT:
                RIGHT = False
            elif event.key == pygame.K_LEFT:
                LEFT = False

        # マウス
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                dragging = True
                pos = Pos(mouse_x, mouse_y)
                click_positions.append([round(pos[0]), round(pos[1])])

        if event.type == pygame.MOUSEBUTTONUP:
            dragging = False
        # if event.type == pygame.MOUSEMOTION:

        if event.type == pygame.MOUSEWHEEL:
            if event.y > 0:  # 左クリック
                a *= 1.1
                b *= 1.1
            elif event.y < 0:  # 右クリック
                a *= 1 / 1.1
                b *= 1 / 1.1

    if dragging:
        print("dragging")
    if UP:
        oy += 3
    if DOWN:
        oy -= 3
    if RIGHT:
        ox -= 3
    if LEFT:
        ox += 3

    # print("load")
    # 時間とフレームの更新
    pygame.display.flip()
    clock.tick(60)


click_positions = np.array(click_positions)


destfile = "../data/test_route.dat"
np.savetxt(destfile, click_positions)
with open(destfile, "r", encoding="utf-8") as f:
    lines = f.readlines()

line_count = len(lines)
lines.insert(0, f"{line_count}\n")

with open(destfile, "w", encoding="utf-8") as f:
    f.writelines(lines)

print(click_positions)

# print(field)
# 終了
pygame.quit()
sys.exit()
