# This project is all done by chun-kuan chih
# -----------------------------
# 蟲蟲1 large bale 2 0 -> 2 3, 2 2 的時候就會斷掉
# 蟲蟲2 新增mid bale的時候會讓scp mcp同時開始跑
# 未解1 車子避開對方
# 更新 0705 把pop放在carrybales
# 除了carrybales的pop 全部comment 未刪除 可以刪除
# -----------------------------
# 先輸入 0 0 4 觀察路徑
# 再輸入 3 1 5 新增障礙物
# 再輸入 0 0 4 觀察路徑
# 會發現這次路徑會避開 3 1 這個位置了
# ------------------------------
import numpy as np
import random
import time
import threading
from collections import deque
from threading import Lock

start = time.perf_counter()

# open file and store in the matrix-
with open('field.txt', 'r') as file:
    data = file.read()  # str object

# replace all the space and new line
data = data.replace("\r", " ").replace("\n", " ")
data = data.replace("\r", "").replace(" ", "")

# create an empty 2d array
kuan = [[0] * 14 for i in range(6)]
# assign value
count = 0
for i in range(0, 6):
    for j in range(0, 14):
        kuan[i][j] = int(data[count])
        count += 1

kuan_str = [[0] * 14 for i in range(6)]
for i in range(0, 6):
    for j in range(0, 14):
        kuan_str[i][j] = str(kuan[i][j])

# global variable
# small
b = []  # possible path for small vehicle
c = []  # drive path for small vehicle
# mid
bb = []  # possible path for mid vehicle
cc = []  # drive path for mid vehicle
# large
bbb = []  # possible path for large vehicle
ccc = []  # drive path for large vehicle
# weed
bbbb = []  # possible path for weed vehicle
cccc = []  # drive path for weed vehicle
# small bales location
sbl = []
# mid bales location
mbl = []
# large bales location
lbl = []
# weed bales loation
wbl = []
input_word = ''
abx = -1
aby = -1
bn = -1

small_field = [[0] * 14 for i in range(6)]
mid_field = [[0] * 14 for i in range(6)]
large_field = [[0] * 14 for i in range(6)]
weed_field = [[0] * 14 for i in range(6)]
running_field = [[0] * 14 for i in range(6)]
for i in range(0, 6):
    for j in range(0, 14):
        running_field[i][j] = str(0)

# Finished


# Main

small_start_x, small_start_y, small_final_x, small_final_y = 3, 3, 1, 12
mid_start_x, mid_start_y, mid_final_x, mid_final_y = 3, 12, 1, 6
large_start_x, large_start_y, large_final_x, large_final_y = 5, 11, 5, 2
weed_start_x, weed_start_y, weed_final_x, weed_final_y = 1, 13, 2, 4

slc = [[-1, -1], [-1, -1]]
mlc = [[-1, -1], [-1, -1]]
llc = [[-1, -1], [-1, -1]]
wlc = [[-1, -1], [-1, -1]]

scp = [[small_start_x], [small_start_y]]
mcp = [[mid_start_x], [mid_start_y]]
lcp = [[large_start_x], [large_start_y]]
wcp = [[weed_start_x], [weed_start_y]]

no = False

#### only calculate the shortest path number, give 1 in the end

# obstacles path simple, return the shortest distance
def small_obstaclepathsimple(grid, startx, starty, finalx, finaly, flag):
    N, M = len(grid), len(grid[0])  # rows, columns
    directions = [[0, 1], [1, 0], [0, -1], [-1, 0]]
    q = deque()
    q.append([startx, starty, 0])  # starting point
    if flag == 0:
        b.append((q[0][0], q[0][1], q[0][2]))
    visited = set()

    while len(q) > 0:
        cr, cc, cdist = q.popleft()  # x y dist
        # # # print(cr, cc)
        # # # print(finalx, finaly)
        if cr == finalx and cc == finaly:  # final position
            return cdist

        # 4 time, directions
        for direction in directions:
            nr, nc = cr + direction[0], cc + direction[1]
            # no duplicate starting point, no obstacles
            if 0 <= nr < N and 0 <= nc < M and (nr, nc) not in visited and kuan[nr][nc] != 5 and not (
                    nr == startx and nc == starty):
                q.append([nr, nc, cdist + 1])
                if flag == 0:
                    b.append([nr, nc, cdist + 1])
                visited.add((nr, nc))

    return -1


# obstacles path simple, return the shortest distance
def mid_obstaclepathsimple(grid, startx, starty, finalx, finaly, flag):
    N, M = len(grid), len(grid[0])  # rows, columns
    directions = [[0, 1], [1, 0], [0, -1], [-1, 0]]
    q = deque()
    q.append([startx, starty, 0])  # starting point
    if flag == 0:
        bb.append((q[0][0], q[0][1], q[0][2]))
    visited = set()

    while len(q) > 0:
        cr, cc, cdist = q.popleft()  # x y dist
        if cr == finalx and cc == finaly:  # final position
            return cdist

        # 4 time, directions
        for direction in directions:
            nr, nc = cr + direction[0], cc + direction[1]
            # no duplicate starting point, no obstacles
            if 0 <= nr < N and 0 <= nc < M and (nr, nc) not in visited and kuan[nr][nc] != 5 and not (
                    nr == startx and nc == starty):
                q.append([nr, nc, cdist + 1])
                if flag == 0:
                    bb.append([nr, nc, cdist + 1])
                visited.add((nr, nc))

    # # print('no way')
    return -1


# obstacles path simple, return the shortest distance
def large_obstaclepathsimple(grid, startx, starty, finalx, finaly, flag):
    N, M = len(grid), len(grid[0])  # rows, columns
    directions = [[0, 1], [1, 0], [0, -1], [-1, 0]]
    q = deque()
    q.append([startx, starty, 0])  # starting point
    if flag == 0:
        bbb.append((q[0][0], q[0][1], q[0][2]))
    visited = set()

    while len(q) > 0:
        cr, cc, cdist = q.popleft()  # x y dist
        if cr == finalx and cc == finaly:  # final position
            return cdist

        # 4 time, directions
        for direction in directions:
            nr, nc = cr + direction[0], cc + direction[1]
            # no duplicate starting point, no obstacles
            if 0 <= nr < N and 0 <= nc < M and (nr, nc) not in visited and kuan[nr][nc] != 5 and not (
                    nr == startx and nc == starty):
                q.append([nr, nc, cdist + 1])
                if flag == 0:
                    bbb.append([nr, nc, cdist + 1])
                visited.add((nr, nc))

    # # print('no way')
    return -1


# obstacles path simple, return the shortest distance
def weed_obstaclepathsimple(grid, startx, starty, finalx, finaly, flag):
    N, M = len(grid), len(grid[0])  # rows, columns
    directions = [[0, 1], [1, 0], [0, -1], [-1, 0]]
    q = deque()
    q.append([startx, starty, 0])  # starting point
    if flag == 0:
        bbbb.append((q[0][0], q[0][1], q[0][2]))
    visited = set()

    while len(q) > 0:
        cr, cc, cdist = q.popleft()  # x y dist
        if cr == finalx and cc == finaly:  # final position
            return cdist

        # 4 time, directions
        for direction in directions:
            nr, nc = cr + direction[0], cc + direction[1]
            # no duplicate starting point, no obstacles
            if 0 <= nr < N and 0 <= nc < M and (nr, nc) not in visited and kuan[nr][nc] != 5 and not (
                    nr == startx and nc == starty):
                q.append([nr, nc, cdist + 1])
                if flag == 0:
                    bbbb.append([nr, nc, cdist + 1])
                visited.add((nr, nc))

    # # print('no way')
    return -1


### Back Track Path

# back track path, show the driven path
def small_backtrackpath(finalx, finaly, total_step):
    c.append([finalx, finaly, total_step])
    # keep doing until reach start position
    while c[len(c) - 1][2] != 0:
        # check all possible paths
        for i in range(len(b)):
            # if you are the last step toward last element's x and y
            if b[i][2] == c[len(c) - 1][2] - 1:
                # if only 1 step, add it to the drive path
                if small_obstaclepathsimple(kuan, b[i][0], b[i][1], c[len(c) - 1][0], c[len(c) - 1][1], 1) == 1:
                    c.append(b[i])
    # from last to the first, reverse it
    c.reverse()
    # # # print('small drive path')
    # # print(c)
    return c

    # show the best route in the field
    for i in range(0, 6):
        for j in range(0, 14):
            small_field[i][j] = str(kuan[i][j])


# back track path, show the driven path
def mid_backtrackpath(finalx, finaly, total_step):
    cc.append([finalx, finaly, total_step])
    # keep doing until reach start position
    while cc[len(cc) - 1][2] != 0:
        # check all possible paths
        for i in range(len(bb)):
            # if you are the last step toward last element's x and y
            if bb[i][2] == cc[len(cc) - 1][2] - 1:
                # if only 1 step, add it to the drive path
                if mid_obstaclepathsimple(kuan, bb[i][0], bb[i][1], cc[len(cc) - 1][0], cc[len(cc) - 1][1], 1) == 1:
                    cc.append(bb[i])
    # from last to the first, reverse it
    cc.reverse()

    # # # print('mid drive path')
    # # print(cc)

    # show the best route in the field
    for i in range(0, 6):
        for j in range(0, 14):
            mid_field[i][j] = str(kuan[i][j])


# back track path, show the driven path
def large_backtrackpath(finalx, finaly, total_step):
    ccc.append([finalx, finaly, total_step])
    # keep doing until reach start position
    while ccc[len(ccc) - 1][2] != 0:
        # check all possible paths
        for i in range(len(bbb)):
            # if you are the last step toward last element's x and y
            if bbb[i][2] == ccc[len(ccc) - 1][2] - 1:
                # if only 1 step, add it to the drive path
                if large_obstaclepathsimple(kuan, bbb[i][0], bbb[i][1], ccc[len(ccc) - 1][0], ccc[len(ccc) - 1][1],
                                            1) == 1:
                    ccc.append(bbb[i])
    # from last to the first, reverse it
    ccc.reverse()

    # show the best route in the field
    for i in range(0, 6):
        for j in range(0, 14):
            large_field[i][j] = str(kuan[i][j])


# back track path, show the driven path
def weed_backtrackpath(finalx, finaly, total_step):
    cccc.append([finalx, finaly, total_step])
    # keep doing until reach start position
    while cccc[len(cccc) - 1][2] != 0:
        # check all possible paths
        for i in range(len(bbbb)):
            # if you are the last step toward last element's x and y
            if bbbb[i][2] == cccc[len(cccc) - 1][2] - 1:
                # if only 1 step, add it to the drive path
                if weed_obstaclepathsimple(kuan, bbbb[i][0], bbbb[i][1], cccc[len(cccc) - 1][0], cccc[len(cccc) - 1][1],
                                           1) == 1:
                    cccc.append(bbbb[i])
    # from last to the first, reverse it
    cccc.reverse()

    # show the best route in the field
    for i in range(0, 6):
        for j in range(0, 14):
            weed_field[i][j] = str(kuan[i][j])


### carry the bales, give 0 if carried
def carrybales(which_bale, lock, *bales):
    global scp, mcp, lcp, wcp, no
    if which_bale == 'small':
        count = 0
        for i in range(len(bales[0])):
            lock.acquire()
            if no == True:
                return -1
            scp = [bales[0][i][0]], [bales[0][i][1]]
            if kuan[bales[0][i][0]][bales[0][i][1]] == 1 and count < 3:
                sbl.pop(sbl.index([bales[0][i][0], bales[0][i][1]]))
                kuan[bales[0][i][0]][bales[0][i][1]] = 0
                count += 1
                print('scp\t', scp, '\tsmall carried!')
            else:
                print('scp\t', scp)
            lock.release()
            time.sleep(0.1)
    elif which_bale == 'mid':
        count = 0
        for i in range(len(bales[0])):
            lock.acquire()
            if no == True:
                return -1
            mcp = [bales[0][i][0]], [bales[0][i][1]]
            if kuan[bales[0][i][0]][bales[0][i][1]] == 2 and count < 2:
                mbl.pop(mbl.index([bales[0][i][0], bales[0][i][1]]))
                kuan[bales[0][i][0]][bales[0][i][1]] = 0
                count += 1
                print('mcp\t', mcp, '\tmid carried!')
            else:
                print('mcp\t', mcp)
            lock.release()
            time.sleep(0.1)
    elif which_bale == 'large':
        count = 0
        for i in range(len(bales[0])):
            lock.acquire()
            if no == True:
                return -1
            lcp = [bales[0][i][0]], [bales[0][i][1]]
            if kuan[bales[0][i][0]][bales[0][i][1]] == 3 and count < 2:
                lbl.pop(lbl.index([bales[0][i][0], bales[0][i][1]]))
                kuan[bales[0][i][0]][bales[0][i][1]] = 0
                count += 1
                print('lcp\t', lcp, '\tlarge carried!')
            else:
                print('lcp\t', lcp)
            lock.release()
            time.sleep(0.1)
    elif which_bale == 'weed':
        count = 0
        for i in range(len(bales[0])):
            lock.acquire()
            if no == True:
                return -1
            wcp = [bales[0][i][0]], [bales[0][i][1]]
            if kuan[bales[0][i][0]][bales[0][i][1]] == 4 and count < 1:
                wbl.pop(wbl.index([bales[0][i][0], bales[0][i][1]]))
                kuan[bales[0][i][0]][bales[0][i][1]] = 0
                count += 1
                print('wcp\t', wcp, '\tweed carried')
            else:
                print('wcp\t', wcp)
            lock.release()
            time.sleep(0.1)
    elif which_bale == 'small_mid':
        count = 0
        for i in range(len(bales[0])):
            lock.acquire()
            if no == True:
                return -1
            scp = [bales[0][i][0]], [bales[0][i][1]]
            if kuan[bales[0][i][0]][bales[0][i][1]] == 2 and count < 1:
                mbl.pop(mbl.index([bales[0][i][0], bales[0][i][1]]))
                kuan[bales[0][i][0]][bales[0][i][1]] = 0
                count += 1
                print('scp\t', scp, '\tmid carried!\t(small)')
            else:
                print('scp\t', scp)
            lock.release()
            time.sleep(0.1)
    elif which_bale == 'large_small':
        count = 0
        for i in range(len(bales[0])):
            lock.acquire()
            if no == True:
                return -1
            lcp = [bales[0][i][0]], [bales[0][i][1]]
            if kuan[bales[0][i][0]][bales[0][i][1]] == 1 and count < 2:
                sbl.pop(sbl.index([bales[0][i][0], bales[0][i][1]]))
                kuan[bales[0][i][0]][bales[0][i][1]] = 0
                count += 1
                print('lcp\t', lcp, '\tsmall carried!')
            else:
                print('lcp\t', lcp)
            lock.release()
            time.sleep(0.1)
    elif which_bale == 'large_mid':
        count = 0
        for i in range(len(bales[0])):
            lock.acquire()
            if no == True:
                return -1
            lcp = [bales[0][i][0]], [bales[0][i][1]]
            if kuan[bales[0][i][0]][bales[0][i][1]] == 2 and count < 1:
                mbl.pop(mbl.index([bales[0][i][0], bales[0][i][1]]))
                kuan[bales[0][i][0]][bales[0][i][1]] = 0
                count += 1
                print('lcp\t', lcp, '\tmid carried!')
            else:
                print('lcp\t', lcp)
            lock.release()
            time.sleep(0.1)

### How many bales are in the field


### Store the bales' location in each list
def baleslocation(which_bale):
    if which_bale == 'small':
        for i in range(6):
            for j in range(14):
                if kuan[i][j] == 1:
                    sbl.append([i, j])
    elif which_bale == 'mid':
        for i in range(6):
            for j in range(14):
                if kuan[i][j] == 2:
                    mbl.append([i, j])
    elif which_bale == 'large':
        for i in range(6):
            for j in range(14):
                if kuan[i][j] == 3:
                    lbl.append([i, j])
    elif which_bale == 'weed':
        for i in range(6):
            for j in range(14):
                if kuan[i][j] == 4:
                    wbl.append([i, j])


### The furthest bale to collection center
def furthestbale(collectionx, collectiony, *bales):
    kuantable = []  # empty list, for storing each bale distance to the collection
    total = len(bales[0])  # how many bales from the input list?
    # calculate the distance and store in the list, distance, x, y
    for i in range(total):
        distance = small_obstaclepathsimple(kuan, collectionx, collectiony, bales[0][i][0], bales[0][i][1], 1)
        kuantable.append([distance, bales[0][i][0], bales[0][i][1]])
    kuantable.sort()  # the shortest will be the first
    kuantable.reverse()  # but I want to furtheest, so I reverse the list
    return kuantable[0][1], kuantable[0][2]  # return the furthest bale x, y


### The closest two bales near the furthest bale

# maybe need to add a nubmer argument
def nearthefurthest(farawaybalex, farawaybaley, number, *bales):
    if number == 1:  # if the remaining bale is equal to 1
        kuantable = []  # empty list, for storing each bale distance to the collection
        total = len(bales[0])  # how many bales from the input list?
        # calculate the distance and store in the list, distance, x, y
        for i in range(total):
            distance = small_obstaclepathsimple(kuan, farawaybalex, farawaybaley, bales[0][i][0], bales[0][i][1], 1)
            kuantable.append([distance, bales[0][i][0], bales[0][i][1]])
        kuantable.sort()  # the shortest will be the first
        # # print(kuantable) # distance, x, y
        return kuantable[0][1], kuantable[0][2]  # return the nearest bale x, y
    elif number == 2:
        kuantable = []  # empty list, for storing each bale distance to the collection
        total = len(bales[0])  # how many bales from the input list?
        # calculate the distance and store in the list, distance, x, y
        for i in range(total):
            distance = small_obstaclepathsimple(kuan, farawaybalex, farawaybaley, bales[0][i][0], bales[0][i][1], 1)
            kuantable.append([distance, bales[0][i][0], bales[0][i][1]])
        kuantable.sort()  # the shortest will be the first
        return kuantable[0][1], kuantable[0][2], kuantable[1][1], kuantable[1][2]  # return the nearset two bales x y


def starttobales(startx, starty, *threebales):
    kuantable = []
    total = len(threebales[0])
    for i in range(total):
        distance = small_obstaclepathsimple(kuan, startx, starty, threebales[0][i][0], threebales[0][i][1], 1)
        kuantable.append([distance, threebales[0][i][0], threebales[0][i][1]])
    kuantable.sort()
    return kuantable


def threebales(small_bale_number, collectionx, collectiony):
    if small_bale_number >= 3:
        buffer = []
        bale1x, bale1y = furthestbale(collectionx, collectiony, sbl)  # find the furthest bale and get the location
        buffer.append([bale1x, bale1y])
        # sbl.pop(sbl.index([bale1x, bale1y]))  # remove the furthest bale from sbl
        bale2x, bale2y, bale3x, bale3y = nearthefurthest(bale1x, bale1y,
                                                         2, sbl)  # find the closest 2 bales to the furthest bale
        buffer.append([bale2x, bale2y])
        # sbl.pop(sbl.index([bale2x, bale2y]))
        # buffer.append([bale3x, bale3y])
        sbl.pop(sbl.index([bale3x, bale3y]))
        return buffer
    else:
        if small_bale_number == 1:
            buffer = []
            bale1x, bale1y = furthestbale(collectionx, collectiony,
                                          sbl)  # find the furthest bale and get the location
            buffer.append([bale1x, bale1y])
            # sbl.pop(sbl.index([bale1x, bale1y]))  # remove the furthest bale from sbl
            return buffer
        elif small_bale_number == 2:
            buffer = []
            bale1x, bale1y = furthestbale(collectionx, collectiony,
                                          sbl)  # find the furthest bale and get the location
            buffer.append([bale1x, bale1y])
            # sbl.pop(sbl.index([bale1x, bale1y]))  # remove the furthest bale from sbl
            bale2x, bale2y, = nearthefurthest(bale1x, bale1y, 1,
                                              sbl)  # find the closest 2 bales to the furthest bale
            buffer.append([bale2x, bale2y])
            # sbl.pop(sbl.index([bale2x, bale2y]))
            return buffer


def twobalesmid(mid_bale_number, collectionx, collectiony):
    if mid_bale_number == 1:  # if only 1 bale remaining
        buffer = []
        # find the furthest bale and get the location
        bale1x, bale1y = furthestbale(collectionx, collectiony, mbl)
        buffer.append([bale1x, bale1y])  # add it to the buffer
        # mbl(mbl.index([bale1x, bale1y]))  # remove the furthest bale from mbl
        return buffer  # return the furthest bale
    elif mid_bale_number == 2:
        buffer = []
        # find the furthest bale and get the location
        bale1x, bale1y = furthestbale(collectionx, collectiony, mbl)
        buffer.append([bale1x, bale1y])  # add it to the buffer
        # mbl(mbl.index([bale1x, bale1y]))  # remove the furthest bale from mbl
        # find the nearest bales to the furthest bale
        bale2x, bale2y, = nearthefurthest(bale1x, bale1y, 1, mbl)
        buffer.append([bale2x, bale2y])
        # mbl(mbl.index([bale2x, bale2y]))
        return buffer  # return furhtest bale and the nearest bale to it


#### Small truck: 3 small / 1 mid
def small_bale(grid, start_x, start_y, final_x, final_y, collection_midx, collection_midy, flag, slock, mlock, lock):
    global no
    print('start small')
    baleslocation('small')  # show all bales location, store in sbl
    print('small bales\t', sbl)
    time.sleep(0.1)
    small = []
    small_each = []
    small_mid = []
    small_mid_each = []
    threebaleslocation = []

    while input_word != 'end':

        # small bale
        small_bale_number = len(sbl)
        while small_bale_number != 0:
            if small_bale_number >= 3:  # > 3
                buffer = []
                small_each = []
                threebaleslocation = threebales(small_bale_number, final_x, final_y)
                buffer = starttobales(start_x, start_y, threebaleslocation)
                if small_obstaclepathsimple(grid, buffer[0][1], buffer[0][2], buffer[1][1], buffer[1][2],
                                            1) + small_obstaclepathsimple(grid, buffer[1][1], buffer[1][2],
                                                                          buffer[2][1], buffer[2][2],
                                                                          1) + small_obstaclepathsimple(grid,
                                                                                                        buffer[2][
                                                                                                            1],
                                                                                                        buffer[2][
                                                                                                            2],
                                                                                                        final_x,
                                                                                                        final_y,
                                                                                                        1) > small_obstaclepathsimple(
                    grid, buffer[0][1], buffer[0][2], buffer[2][1], buffer[2][2], 1) + small_obstaclepathsimple(
                    grid, buffer[2][1], buffer[2][2], buffer[1][1], buffer[1][2], 1) + small_obstaclepathsimple(
                    grid, buffer[1][1], buffer[1][2], final_x, final_y, 1):
                    threebaleslocation = [[buffer[0][1], buffer[0][2]], [buffer[2][1], buffer[2][2]],
                                          [buffer[1][1], buffer[1][2]]]
                # start to first bale
                ssd = small_obstaclepathsimple(grid, scp[0][0], scp[1][0], threebaleslocation[0][0],
                                               threebaleslocation[0][1], flag)
                if ssd != -1:
                    small_backtrackpath(threebaleslocation[0][0], threebaleslocation[0][1], ssd)
                # # print(c)
                for i in range(len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # first bale to second bale
                ssd = small_obstaclepathsimple(grid, threebaleslocation[0][0], threebaleslocation[0][1],
                                               threebaleslocation[1][0], threebaleslocation[1][1], flag)
                if ssd != -1:
                    small_backtrackpath(threebaleslocation[1][0], threebaleslocation[1][1], ssd)
                # # print(c)
                for i in range(1, len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(1, len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # second bale to third bale
                ssd = small_obstaclepathsimple(grid, threebaleslocation[1][0], threebaleslocation[1][1],
                                               threebaleslocation[2][0],
                                               threebaleslocation[2][1], flag)
                if ssd != -1:
                    small_backtrackpath(threebaleslocation[2][0], threebaleslocation[2][1], ssd)
                # # print(c)
                for i in range(1, len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(1, len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # third bale to collection center
                ssd = small_obstaclepathsimple(grid, threebaleslocation[2][0], threebaleslocation[2][1], final_x,
                                               final_y, flag)
                if ssd != -1:
                    small_backtrackpath(final_x, final_y, ssd)
                # # print(c)
                for i in range(1, len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(1, len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # carry the bales
                carrybales('small', lock, small_each)
                no = False
                # update the small bales number
                print('small path, carry 3 bales', small_each)
            elif small_bale_number == 2:
                buffer = []
                small_each = []
                threebaleslocation = threebales(small_bale_number, final_x, final_y)
                # start to first bale
                ssd = small_obstaclepathsimple(grid, scp[0][0], scp[1][0], threebaleslocation[0][0],
                                               threebaleslocation[0][1], flag)
                if ssd != -1:
                    small_backtrackpath(threebaleslocation[0][0], threebaleslocation[0][1], ssd)
                # # print(c)
                for i in range(len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # first bale to second bale
                ssd = small_obstaclepathsimple(grid, threebaleslocation[0][0], threebaleslocation[0][1],
                                               threebaleslocation[1][0], threebaleslocation[1][1], flag)
                if ssd != -1:
                    small_backtrackpath(threebaleslocation[1][0], threebaleslocation[1][1], ssd)
                # # print(c)
                for i in range(1, len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(1, len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # second bale to collection center
                ssd = small_obstaclepathsimple(grid, threebaleslocation[1][0], threebaleslocation[1][1], final_x,
                                               final_y, flag)
                if ssd != -1:
                    small_backtrackpath(final_x, final_y, ssd)
                # # print(c)
                for i in range(1, len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(1, len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # carry the bales
                carrybales('small', lock, small_each)
                no = False
                # update the small bales number
                print('small path, carry 2 bales', small_each)
            elif small_bale_number == 1:
                buffer = []
                small_each = []
                threebaleslocation = threebales(small_bale_number, final_x, final_y)
                # start to first bale
                ssd = small_obstaclepathsimple(grid, scp[0][0], scp[1][0], threebaleslocation[0][0],
                                               threebaleslocation[0][1], flag)
                if ssd != -1:
                    small_backtrackpath(threebaleslocation[0][0], threebaleslocation[0][1], ssd)
                # # print(c)
                for i in range(len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # second bale to collection center
                ssd = small_obstaclepathsimple(grid, threebaleslocation[0][0], threebaleslocation[0][1], final_x,
                                               final_y, flag)
                if ssd != -1:
                    small_backtrackpath(final_x, final_y, ssd)
                # # print(c)
                for i in range(1, len(c)):
                    small.append([c[i][0], c[i][1]])
                for i in range(1, len(c)):
                    small_each.append([c[i][0], c[i][1]])
                c.clear()
                b.clear()
                # carry the bales
                carrybales('small', lock, small_each)
                no = False
                # update the small bales number
                print('small path, carry 1 bale', small_each)
                # # print(threebaleslocation)
                # # print('zxczxc', small_each)
            time.sleep(0.1)
            small_bale_number = len(sbl)
        baleslocation('small')

        # mid bale
        mid_bale_number = len(mbl)
        while mid_bale_number != 0:
            small_mid_each = []
            # # print('\nmid remaining number', mid_bale_number)
            # find the nearest bale location and store in the variable
            # # print('mbl before', mbl)
            # distance, x, y
            # show all the bales location with how many steps
            bale1x, bale1y = nearthefurthest(collection_midx, collection_midy, 1, mbl)
            # mbl(mbl.index([bale1x, bale1y]))
            # # print('mbl after', mbl)
            # start to first bale
            # # print('small collection\t', final_x, final_y)
            # # print('bale location\t\t', bale1x, bale1y)
            # # print('mid collection\t\t', collection_midx, collection_midy)
            # small collection center to first bale
            msd = small_obstaclepathsimple(grid, scp[0][0], scp[1][0], bale1x, bale1y, flag)
            if msd != -1:
                small_backtrackpath(bale1x, bale1y, msd)
            for i in range(len(c)):
                small_mid.append([c[i][0], c[i][1]])
            for i in range(len(c)):
                small_mid_each.append([c[i][0], c[i][1]])
            c.clear()
            b.clear()
            # second bale to mid collection center
            msd = small_obstaclepathsimple(grid, bale1x, bale1y, collection_midx, collection_midy, flag)
            if msd != -1:
                small_backtrackpath(collection_midx, collection_midy, msd)
            for i in range(1, len(c)):
                small_mid.append([c[i][0], c[i][1]])
            for i in range(1, len(c)):
                small_mid_each.append([c[i][0], c[i][1]])
            c.clear()
            b.clear()
            carrybales('small_mid', lock, small_mid_each)
            no = False
            print('small car carry mid bale from current position', small_mid_each)
            time.sleep(0.1)
            mlock.acquire()
            mid_bale_number = len(mbl)
            mlock.release()
        baleslocation('mid')

    print('Exiting small vehicle')


#### Mid truck: 2 mid
def mid_bale(grid, start_x, start_y, final_x, final_y, flag, mlock, lock):
    global no
    print('start mid')
    baleslocation('mid')
    print('mid bales\t', mbl)
    time.sleep(0.1)

    mid = []
    mid_each = []
    while input_word != 'end':

        # mid bale
        mid_bale_number = len(mbl)
        while mid_bale_number != 0:
            if mid_bale_number >= 2:
                mid_each = []
                b1x, b1y, b2x, b2y = nearthefurthest(start_x, start_y, 2, mbl)
                # mbl(mbl.index([b1x, b1y]))
                # mbl(mbl.index([b2x, b2y]))
                # last position to first bale
                msd = mid_obstaclepathsimple(grid, mcp[0][0], mcp[1][0], b1x, b1y, flag)
                if msd != -1:
                    mid_backtrackpath(b1x, b1y, msd)
                for i in range(len(cc)):
                    mid.append([cc[i][0], cc[i][1]])
                for i in range(len(cc)):
                    mid_each.append([cc[i][0], cc[i][1]])
                cc.clear()
                bb.clear()
                # first bale to second bale
                msd = mid_obstaclepathsimple(grid, b1x, b1y, b2x, b2y, flag)
                if msd != -1:
                    mid_backtrackpath(b2x, b2y, msd)
                for i in range(1, len(cc)):
                    mid.append([cc[i][0], cc[i][1]])
                for i in range(1, len(cc)):
                    mid_each.append([cc[i][0], cc[i][1]])
                cc.clear()
                bb.clear()
                # second bale to mid collection center
                msd = mid_obstaclepathsimple(grid, b2x, b2y, final_x, final_y, flag)
                if msd != -1:
                    mid_backtrackpath(final_x, final_y, msd)
                for i in range(1, len(cc)):
                    mid.append([cc[i][0], cc[i][1]])
                for i in range(1, len(cc)):
                    mid_each.append([cc[i][0], cc[i][1]])
                cc.clear()
                bb.clear()
                # carry the bales
                carrybales('mid', lock, mid_each)
                no = False
                # update the small bale number
                print('mid path, carry 2 bales', mid_each)
            elif mid_bale_number == 1:
                mid_each = []
                b1x, b1y = nearthefurthest(final_x, final_y, 1, mbl)
                # mbl(mbl.index([b1x, b1y]))
                # to first bale
                msd = mid_obstaclepathsimple(grid, mcp[0][0], mcp[1][0], b1x, b1y,
                                             flag)
                if msd != -1:
                    mid_backtrackpath(b1x, b1y, msd)
                for i in range(len(cc)):
                    mid.append([cc[i][0], cc[i][1]])
                for i in range(len(cc)):
                    mid_each.append([cc[i][0], cc[i][1]])
                cc.clear()
                bb.clear()
                # first bale to mid collection center
                msd = mid_obstaclepathsimple(grid, b1x, b1y, final_x, final_y, flag)
                if msd != -1:
                    mid_backtrackpath(final_x, final_y, msd)
                for i in range(1, len(cc)):
                    mid.append([cc[i][0], cc[i][1]])
                for i in range(1, len(cc)):
                    mid_each.append([cc[i][0], cc[i][1]])
                cc.clear()
                bb.clear()
                # carry the bales
                carrybales('mid', lock, mid_each)
                no = False
                # update the small bale number
                print('mid path, carry 2 bales', mid_each)
            time.sleep(0.1)
            mlock.acquire()
            mid_bale_number = len(mbl)
            mlock.release()
        baleslocation('mid')

    print('Exiting mid vehicle')


#### Large truck:  2 small / 1 mid / 1 big
def large_bale(grid, start_x, start_y, final_x, final_y, collection_midx, collection_midy, collection_smallx,
               collection_smally, flag, slock, mlock, llock, lock):
    global no
    print('start large')
    baleslocation('large')
    print('large bales\t', lbl)
    time.sleep(0.1)
    large = []
    large_mid = []
    large_each = []
    large_small = []
    large_small_each = []

    while input_word != 'end':

        # carry large bales
        large_bale_number = len(lbl)
        while large_bale_number != 0:
            large_each = []
            bale1x, bale1y = nearthefurthest(final_x, final_y, 1, lbl)
            # lbl(lbl.index([bale1x, bale1y]))
            # to first bale
            lsd = large_obstaclepathsimple(grid, lcp[0][0], lcp[1][0], bale1x, bale1y, flag)
            if lsd != -1:
                large_backtrackpath(bale1x, bale1y, lsd)
            for i in range(len(ccc)):
                large.append([ccc[i][0], ccc[i][1]])
            for i in range(len(ccc)):
                large_each.append([ccc[i][0], ccc[i][1]])
            ccc.clear()
            bbb.clear()
            # second bale to large collection center
            lsd = large_obstaclepathsimple(grid, bale1x, bale1y, final_x, final_y, flag)
            if lsd != 1:
                large_backtrackpath(final_x, final_y, lsd)
            for i in range(1, len(ccc)):
                large.append([ccc[i][0], ccc[i][1]])
            for i in range(1, len(ccc)):
                large_each.append([ccc[i][0], ccc[i][1]])
            ccc.clear()
            bbb.clear()
            # carry the bales
            carrybales('large', lock, large_each)
            no = False
            # update the large bale nubmer
            print('large path', large_each)
            large_bale_number = len(lbl)
        baleslocation('large')

        # carry small bales
        small_bale_number = len(sbl)
        while small_bale_number != 0:
            if small_bale_number >= 2:
                large_small_each = []
                b1x, b1y, b2x, b2y = nearthefurthest(collection_smallx, collection_smally, 2, sbl)
                # sbl.pop(sbl.index([b1x, b1y]))
                # sbl.pop(sbl.index([b2x, b2y]))
                # to first bale
                ssd = large_obstaclepathsimple(grid, lcp[0][0], lcp[1][0], b1x, b1y, flag)
                if ssd != -1:
                    large_backtrackpath(b1x, b1y, ssd)
                for i in range(len(ccc)):
                    large_small.append([ccc[i][0], ccc[i][1]])
                for i in range(len(ccc)):
                    large_small_each.append([ccc[i][0], ccc[i][1]])
                ccc.clear()
                bbb.clear()
                # first bale to second bale
                ssd = large_obstaclepathsimple(grid, b1x, b1y, b2x, b2y, flag)
                if ssd != -1:
                    large_backtrackpath(b2x, b2y, ssd)
                for i in range(1, len(ccc)):
                    large_small.append([ccc[i][0], ccc[i][1]])
                for i in range(1, len(ccc)):
                    large_small_each.append([ccc[i][0], ccc[i][1]])
                ccc.clear()
                bbb.clear()
                # second bale to small collection center
                ssd = large_obstaclepathsimple(grid, b2x, b2y, collection_smallx, collection_smally, flag)
                if ssd != -1:
                    large_backtrackpath(collection_smallx, collection_smally, ssd)
                for i in range(1, len(ccc)):
                    large_small.append([ccc[i][0], ccc[i][1]])
                for i in range(1, len(ccc)):
                    large_small_each.append([ccc[i][0], ccc[i][1]])
                ccc.clear()
                bbb.clear()
                # carry the bales
                carrybales('small', lock, large_small_each)
                no = False
                # update the small bale number
                small_bale_number = len(sbl)
                print('large carry 2 small bales from current position', large_small_each)
            elif small_bale_number == 1:
                large_small_each = []
                b1x, b1y = nearthefurthest(collection_smallx, collection_smally, 1, sbl)
                # sbl.pop(sbl.index([b1x, b1y]))
                # large collection to first bale
                ssd = large_obstaclepathsimple(grid, final_x, final_y, b1x, b1y,
                                               flag)
                if ssd != -1:
                    large_backtrackpath(b1x, b1y, ssd)
                for i in range(len(ccc)):
                    large_small.append([ccc[i][0], ccc[i][1]])
                for i in range(len(ccc)):
                    large_small_each.append([ccc[i][0], ccc[i][1]])
                ccc.clear()
                bbb.clear()
                # first bale to small collection center
                ssd = large_obstaclepathsimple(grid, b1x, b1y, collection_smallx,
                                               collection_smally, flag)
                if ssd != -1:
                    large_backtrackpath(collection_smallx, collection_smally, ssd)
                for i in range(1, len(ccc)):
                    large_small.append([ccc[i][0], ccc[i][1]])
                for i in range(1, len(ccc)):
                    large_small_each.append([ccc[i][0], ccc[i][1]])
                ccc.clear()
                bbb.clear()
                # carry the bales
                carrybales('small', lock, large_small_each)
                no = False
                # update the small bale number
                small_bale_number = len(sbl)
                print('large carry 1 small bale from current position,', large_small_each)
            baleslocation('small')

        # carry mid bales
        mid_bale_number = len(mbl)
        while mid_bale_number != 0:
            large_mid_each = []
            bale1x, bale1y = nearthefurthest(collection_midx, collection_midy, 1, mbl)
            # mbl(mbl.index([bale1x, bale1y]))
            msd = large_obstaclepathsimple(grid, lcp[0][0], lcp[1][0], bale1x, bale1y, flag)
            if msd != -1:
                large_backtrackpath(bale1x, bale1y, msd)
            for i in range(len(ccc)):
                large_mid.append([ccc[i][0], ccc[i][1]])
            for i in range(len(ccc)):
                large_mid_each.append([ccc[i][0], ccc[i][1]])
            ccc.clear()
            bbb.clear()
            # second bale to collection center
            msd = large_obstaclepathsimple(grid, bale1x, bale1y, collection_midx, collection_midy, flag)
            if msd != -1:
                large_backtrackpath(collection_midx, collection_midy, msd)
            for i in range(1, len(ccc)):
                large_mid.append([ccc[i][0], ccc[i][1]])
            for i in range(1, len(ccc)):
                large_mid_each.append([ccc[i][0], ccc[i][1]])
            ccc.clear()
            bbb.clear()
            # carry the bales
            carrybales('mid', lock, large_mid_each)
            no = False
            # update the mid bale nubmer
            mid_bale_number = len(mbl)
            print('large carry mid bale from current position', large_mid_each)
        baleslocation('mid')



    print('Exiting large vehicle')


#### weed: 1 weed
def weed_bale(grid, start_x, start_y, final_x, final_y, flag, lock):
    global no
    print('strart weed')
    baleslocation('weed')
    print('weed bales\t', wbl)
    time.sleep(0.1)
    weed = []
    weed_each = []

    while input_word != 'end':

        # carry weed bale
        weed_bale_number = len(wbl)
        while weed_bale_number != 0:
            weed_each = []
            bale1x, bale1y = nearthefurthest(final_x, final_y, 1, wbl)
            # wbl(wbl.index([bale1x, bale1y]))
            wsd = weed_obstaclepathsimple(grid, wcp[0][0], wcp[1][0], bale1x, bale1y, flag)
            if wsd != -1:
                weed_backtrackpath(bale1x, bale1y, wsd)
            for i in range(len(cccc)):
                weed.append([cccc[i][0], cccc[i][1]])
            for i in range(len(cccc)):
                weed_each.append([cccc[i][0], cccc[i][1]])
            cccc.clear()
            bbbb.clear()
            # # second bale to collection center
            wsd = weed_obstaclepathsimple(grid, bale1x, bale1y, final_x, final_y, flag)
            if wsd != -1:
                weed_backtrackpath(final_x, final_y, wsd)
            for i in range(1, len(cccc)):
                weed.append([cccc[i][0], cccc[i][1]])
            for i in range(1, len(cccc)):
                weed_each.append([cccc[i][0], cccc[i][1]])
            cccc.clear()
            bbbb.clear()
            carrybales('weed', lock, weed_each)
            no = False
            print('weed path', weed_each)
            weed_bale_number = len(wbl)
        baleslocation('weed')

    print('Exiting weed vehicle')


def addbales():
    global input_word, abx, aby, bn, no
    input_word = input()
    while input_word != 'end':
        abx = int(input_word[0:input_word.index(' ')])
        aby = int(input_word[input_word.index(' ') + 1:input_word.index(' ', input_word.index(' ') + 1, len(input_word))])
        bn = int(input_word[len(input_word) - 1])
        print('location', abx, aby)
        if bn == 1:
            print('small bale')
        if bn == 2:
            print('mid bale')
        if bn == 3:
            print('large bale')
        if bn == 4:
            print('weed bale')
        if bn == 5:
            print('obstacles!')
        if kuan[abx][aby] != 0:
            print('you cannot place new bales or new obstalces here')
            print('Please enter a new location')
        else:
            kuan[abx][aby] = bn
        if bn == 5:
            no = True
        input_word = input()
        no = False


slock = Lock()
mlock = Lock()
llock = Lock()
lock = Lock()

# create threads
ts = threading.Thread(target=small_bale,
                      args=(
                          kuan, small_start_x, small_start_y, small_final_x, small_final_y, mid_final_x, mid_final_y, 0,
                          slock, mlock, lock,))
tm = threading.Thread(target=mid_bale, args=(kuan, mid_start_x, mid_start_y, mid_final_x, mid_final_y, 0, mlock, lock,))
tl = threading.Thread(target=large_bale,
                      args=(kuan, large_start_x, large_start_y, large_final_x, large_final_y, mid_final_x, mid_final_y,
                            small_final_x, small_final_y, 0, slock, mlock, llock, lock,))
tw = threading.Thread(target=weed_bale, args=(kuan, weed_start_x, weed_start_y, weed_final_x, weed_final_y, 0, lock,))
addbales = threading.Thread(target=addbales)

print('small\tstart position\t\t', small_start_x, small_start_y)
print('mid\t\tstart position\t\t', mid_start_x, mid_start_y)
print('large\tstart position\t\t', large_start_x, large_start_y)
print('weed\tstart position\t\t', weed_start_x, weed_start_y)
print('')
print('small\tcollection center\t', small_final_x, small_final_y)
print('mid\t\tcollection center\t', mid_final_x, mid_final_y)
print('large\tcollection center\t', large_final_x, large_final_y)
print('weed\tcollection center\t', weed_final_x, weed_final_y)
print('')

# start threads
ts.start()
tm.start()
tl.start()
tw.start()
addbales.start()

ts.join()
tm.join()
tl.join()
tw.join()
addbales.join()

finish = time.perf_counter()
print(f'Finished in {round(finish - start, 2)} seconds(s)')

print('Done')