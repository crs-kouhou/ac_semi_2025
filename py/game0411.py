import pygame
import sys
import numpy as np
import time
#初期化
pygame.init()

font = pygame.font.SysFont("Arial", 24)
#サイズ
width, height = 1000, 1000
screen = pygame.display.set_mode((width,height))

pygame.display.set_caption('gragh')
clock = pygame.time.Clock()
#color
white=(255,255,255)
black=(0,0,0)

#パラメータ
ox=round(width/3)
oy=round(height*4/5)
def O(x,y):#画面でのピクセルとしての座標
    return pygame.draw.circle(screen,black, (x,y) ,5)

#拡大率    
a=1/10
b=1/10
s=0
def X():
    return np.linspace(-ox/a,(width-ox)/a,width)
def Y():
    return np.linspace(oy/b,(height+oy)/b,height)
    
#点を打つ
def P(x,y,c,size):
    return pygame.draw.circle(screen,c,(ox+a*x,oy-b*y),size)
def Pos(x,y):#ピクセルを座標にする
    return ((x-ox)/a,(-y+oy)/b) 
#x,yのリスト表示
def Ps(x,y,c,size):
    for i in range(len(x)):
        pygame.draw.circle(screen,c,(ox+a*x[i],oy-b*y[i]),size)

def Polygon(XYlist,c):
    for i in range(XYlist.shape[1]):
            XYlist[0][i]=XYlist[0][i]*a+ox
            XYlist[1][i]=XYlist[1][i]*(-b)+oy
            
    XYlist=XYlist.T
    pygame.draw.polygon(screen,c,XYlist,5)


def Fn(x,y,c):
    for i in range(len(x)):
        if i>0:
            pygame.draw.line(screen,c,(ox+a*x[i-1],oy-b*y[i-1]),(ox+a*x[i],oy-b*y[i]),2)
        
        
def Line(x_1,y_1,x_2,y_2,c):
        pygame.draw.line(screen,c,(ox+a*x_1,oy-b*y_1,ox+a*x_2,oy-b*y_2),2)    
        

field=np.load('spring25.npy')
       
        
        
def Lines(x,c):#xは線分のリスト
    for i in range(len(x)):
        pygame.draw.line(screen,c,(ox+a*x[i][0],oy-b*x[i][1]),(ox+a*x[i][2],oy-b*x[i][3]),2)
    


def ConvertPoints(LINES): 
  POINTS=[]
  n=1/50
  for li in LINES:
      ld=np.sqrt((li[0]-li[2])**2+(li[1]-li[3])**2)
      pl=np.array([np.linspace(li[0],li[2],round(n*ld+1)) ,np.linspace(li[1],li[3],round(n*ld+1))]).T
      for p in pl:
          POINTS.append(p)
  POINTS=np.array(POINTS).T
  return POINTS

click_positions=[]
start_ticks = pygame.time.get_ticks()

load_ticks =start_ticks
running =True
dragging = False
LEFT=False
RIGHT=False
UP=False
DOWN=False

###シミュ

pi=np.pi

#robot
roboav=np.array([[0],[0]])
robox=np.array([0,-1/3,-1,-1,-1,-1/3,0])
roboy=np.array([0,   1.,  1,0,-1  ,-1,0])/2
roboi=np.ones(robox.shape)

robox*=400
roboy*=400

#平均を求めて平行移動
#重心ではないが
xg=np.average(robox)
yg=np.average(roboy)

robox -=xg
roboy -=yg

robovec=np.array([robox,roboy])
robormax=max(np.sqrt(robox**2+roboy**2))


def ITM(x,y,theta,veclists):   #同次変換行列による計算 identity transformation matrix
  #veclists=[[xlocal0,xlocal1,xlocal2,...],[ylocal0,ylocal1,ylocal2,...]]　local座標の点群である。
  #local座標veclistsは(x,y,theta)だけずれている。 veclists_global を示す

    a       =  np.array([[np.cos(theta),-np.sin(theta), x],
                       [np.sin(theta), np.cos(theta), y],
                       [            0,             0, 1]])
    
    aa=np.vstack((veclists, np.ones(veclists.shape[1])))
    return   np.array([np.dot(a,aa)[0],np.dot(a,aa)[1]])

def mod_pi(x):# x を 2π で割ったあまり
    remainder = np.mod(x, 2 * np.pi)
    if remainder > np.pi:# あまりが π を超えた場合、-π から π の範囲に収める
        remainder -= 2 * np.pi

    return remainder

lines=np.load('spring25.npy')
#lines0=lines


def lidarlist(x,y,th,lines0):#linesをグローバル変数?
  #LiDARのデータをclosestlistとする。
  #距離だけ考える
    thetas=np.linspace(-1.5,1.5,80)
    R=20000
    closestlist=np.zeros(len(thetas))
    for i in range(len(thetas)):
        xrobo, yrobo, xbeem, ybeem =x,y,x+R*np.cos(th+thetas[i]),y+R*np.sin(th+thetas[i])
        closest_distance0= np.sqrt((xbeem-xrobo)**2+(ybeem-yrobo)**2)
        for line in lines0:
            
            x1, y1, x2, y2 = line
            
            denominator = (x1 - x2) * (yrobo - ybeem) - (y1 - y2) * (xrobo - xbeem)
            if denominator == 0:
                continue
                
            t = ((x1 - xrobo) * (yrobo - ybeem) - (y1 - yrobo) * (xrobo - xbeem)) / denominator
            u = -((x1 - x2) * (y1 - yrobo) - (y1 - y2) * (x1 - xrobo)) / denominator

            if 0 <= t <= 1 and 0 <= u <= 1:
                intersection_x = x1 + t * (x2 - x1)
                intersection_y = y1 + t * (y2 - y1)
                robo_wall_distance = np.sqrt( (intersection_x-xrobo)**2+(intersection_y-yrobo)**2)
                if (robo_wall_distance<=closest_distance0):
                    closest_distance0 = robo_wall_distance
                else:continue
            else:continue
            
        Lnoise=(1+0.03*np.random.uniform(-1,1))
        closestlist[i]=(Lnoise*closest_distance0)
        
    #LiDARの極座標のデータを直交座標lasermapに変換する。
    X=closestlist*np.cos(thetas)
    Y=closestlist*np.sin(thetas)
    return np.array([X,Y])






def ConvertPoints(LINES):#fieldを点群に#使ってない
  POINTS=[]
  n=1/10
  for li in LINES:
      ld=np.sqrt((li[0]-li[2])**2+(li[1]-li[3])**2)
      pl=np.array([np.linspace(li[0],li[2],round(n*ld+1)) ,np.linspace(li[1],li[3],round(n*ld+1))]).T
      for p in pl:
          POINTS.append(p)
  POINTS=np.array(POINTS).T
  return POINTS

def Lines2(LINES):
    LINES_2=[]
    for i in range(LINES.shape[0]):
        LINES_2.append([LINES[i][0],LINES[i][1],1])
        LINES_2.append([LINES[i][2],LINES[i][3],1])
    LINES_2=np.array(LINES_2)
    return LINES_2
lines2=Lines2(lines)


LINES2=lines2

def icp(x,y,th,LiDAR):#icpマッチングの関数　
  pi=np.pi
  ##############################################################################
  #フィールドの点群はfieldmap　　(x0,y0,th)にあわせる
  R00=np.array([[np.cos(th), -np.sin(th)],
                [np.sin(th),np.cos(th)]])
  p00=[x,y]

  Rtp00=np.dot(R00.T,p00)

  transmatrix_field    =np.array([[ np.cos(th), np.sin(th), -Rtp00[0] ],
                                  [-np.sin(th), np.cos(th),  -Rtp00[1]],
                                  [           0,           0,          1]])
  #th0

  # 同次座標系に変換（行を追加）計算
  transformed = np.dot(transmatrix_field,LINES2.T)

  LINES2t=transformed.T
  A=LiDAR.T

  totalTrans=np.eye(3)
    
  for i in range(18):
    NEARLIST=[]
    for point in A:
      x0,y0=point[0],point[1]

      Short=float('inf')
      for i in range(int(LINES2t.shape[0]/2)) :
        x1,y1,x2,y2 = LINES2t[2*i][0],LINES2t[2*i][1],LINES2t[2*i+1][0],LINES2t[2*i+1][1]

        if (x0-x1)*(x2-x1)+(y0-y1)*(y2-y1)<0:
          xi,yi=x1,y1
        elif (x0-x2)*(x2-x1)+(y0-y2)*(y2-y1)>0:
          xi,yi=x2,y2
        else:
          AI = ((x0-x1)*(x2-x1)+(y0-y1)*(y2-y1))/((x2-x1)**2+(y2-y1)**2)
          xi=x1+AI*(x2-x1)
          yi=y1+AI*(y2-y1)

        di2=(x0-xi)**2+(y0-yi)**2
        if di2 < Short:
          Short = di2
          Xnear=xi
          Ynear=yi
      NEARLIST.append([Xnear,Ynear])


    B=np.array(NEARLIST) #出力は[[x,y],[],[],...]
    # AとBでマッチング

    mean_A=np.mean(A, axis=0)
    mean_B=np.mean(B, axis=0)

    A_c=A-mean_A
    B_c=B-mean_B

    H=np.dot(A_c.T,B_c)

    U, S, Vh=np.linalg.svd(H)

    R= np.dot(Vh.T,U.T)


    if R[0][1]*R[1][0]>0:

      R=np.eye(2)

    t=mean_B-np.dot(R, mean_A)

    Transmatrix  =np.array([[R[0][0],R[0][1],t[0]],
                            [R[1][0],R[1][1],t[1]],
                            [      0,      0,   1]])

    totalTrans= np.dot(Transmatrix,totalTrans)


    new_A3 =np.dot(Transmatrix,np.vstack((A.T,np.ones(A.T.shape[1]))))
    
    new_At =new_A3[:2, :]
    A =new_At.T

    #################
    #totalTransは3x3
  xi=totalTrans[0][2]
  yi=totalTrans[1][2]
  thi=np.arctan2(totalTrans[1][0], totalTrans[0][0])

  calc=np.array([x+xi*np.cos(th)-yi*np.sin(th), y+xi*np.sin(th)+yi*np.cos(th),(th+thi)])

  chx = calc[0]-x
  chy = calc[1]-y
  chth= abs(calc[2]-th)
  change  = np.sqrt(chx**2+chy**2 )


  if (change<500)and(chth<pi/2):
    return calc
  else:
    return np.array([x,y,th])
    #############################################################################

lines=np.load('spring25.npy')
lines2=Lines2(lines)
route=np.load('spring25route1.npy').T

#############################経路完成　roure=[[,,,],[,,,]]

#初期条件
#残さない記録

# inp = np.array(input().split(), dtype=float)
x0, y0, th0 = 2000,500,0
xp, yp, thp = x0,y0,th0
dav=ITM(xp,yp,thp,roboav)#[[x0],[y0],[1]]　#davの型に注意

xr =x0+np.random.uniform(-1,1)
yr =y0+np.random.uniform(-1,1)
thr=th0+0.03*np.random.uniform(-1,1)

lasermap=lidarlist(xr,yr,thr,lines)
Posi=icp(xp,yp,thp,lasermap)

xc  = Posi[0]
yc  = Posi[1]
thc = Posi[2]

xythr=[]
xythp=[]
xythc=[]

xythr.append([xr,yr,thr])
xythp.append([xp,yp,thp])
xythc.append([xc,yc,thc])




while running:
    
    screen.fill(white)#まずは白塗り
    game_time = pygame.time.get_ticks() - start_ticks
    #print(game_time)
    elapsed_time = pygame.time.get_ticks() - load_ticks  #更新時間
    if elapsed_time > 1000:  # 1秒経ったら更新
        load_ticks = pygame.time.get_ticks()

            
    #pygame.draw.circle(screen,black,  (round(width/3),(round(height*2/3)))  ,3)
    P(0,0,black,3)
    #LINES
    Lines(field,black)

    # スタート
    start_time = time.time()
    
    
    head  = 600
    dhead = head/4
    distances=np.zeros(len(route[0]))
    near  = float('inf')
    nearindex=0
    # xnear = float('inf')
    # ynear = float('inf')
    thnear=3.
    cn=0

    ##################
    for i in range(len(route[0])):

        distances[i]=np.sqrt((route[0][i]-xc)**2+(route[1][i]-yc)**2)

        if (head<distances[i])and(distances[i]<near)and(abs ( mod_pi( np.arctan2(route[1][i]-yc,route[0][i]-xc) -thc) )<1.):
            near=distances[i]
            nearindex=i
            # xnear=route[0][i]
            # ynear=route[1][i]
            thnear= np.arctan2(route[1][i]-yc,route[0][i]-xc) 
            alpha=thnear-thc                                  
            cn+=1

    if (cn ==0):
        print("out")
        #head/=3
        
        for i in range(len(route[0])):
            if (head<distances[i])and(distances[i]<near):
                near=distances[i]
                # xnear=route[0][i]
                # ynear=route[1][i]
                thnear= np.arctan2(route[1][i]-yc,route[0][i]-xc) 
                alpha=thnear-thc                                  
                cn+=1 
        #head*=3
                       
    ############次の予想座標を計算

    dr = dhead
    dth =alpha
    
    #処理
    dx,dy     =  dr*np.cos(dth),dr*np.sin(dth)    #変化量　#動く前に+dth動いて+dr進むと考える(動いた後の姿勢が +dthだが)
    dvec =np.array([[dx],[dy]])
    dav=ITM(xc,yc,thc,dvec)  
    
    xp=dav[0][0]
    yp=dav[1][0]
    thp = thc+dth
    
    ##############実際
    drr = dr*(1+0.3*np.random.uniform(-1,1))
    dthr = dth*(1+0.05*np.random.uniform(-1,1))+0.3*np.random.uniform(-1,1)
    #処理
    dxr,dyr   =  drr*np.cos(dthr),drr*np.sin(dthr)   #変化量　#動く前に+dth動いて+dr進むと考える(動いた後の姿勢が +dthだが)
    dvecr =np.array([[dxr],[dyr]])
    davr  =ITM(xr,yr,thr,dvecr)    #dav=[[x_new],[y_new],[1]]
         
    xr,yr,thr = davr[0][0],davr[1][0], thr+dthr
    ##移動したとする
    #入力値(xp,yp,thp,lasermap,lines2) 出力(xythlist)
    #今の座標をicpで求める

    lasermap=lidarlist(xr,yr,thr,lines)
    
    Posi=icp(xp,yp,thp,lasermap)
    xc,yc,thc = Posi[0],Posi[1],Posi[2]
    
    #時間を測る
    end_time = time.time()
    execution_time = end_time - start_time
    
    xythr.append([xr,yr,thr])
    xythp.append([xp,yp,thp])
    xythc.append([xc,yc,thc])


    print(f"実行時間: {execution_time}秒")
    
    
    #############################################################
    ########################################################
    #表示
    # (ox+a*x,oy-b*y)
    
    
    roboposi=ITM(xr,yr,thr,robovec)
    
    Polygon(roboposi,(0,0,255))
    
    lidar = ITM(xc,yc,thc,lasermap)
    Ps(lidar[0],lidar[1],(255,0,0),5)
    P(xr,yr,(255,0,0),3)
    P(xp,yp,(0,255,255),3)
    P(xc,yc,(0,0,255),3)#blue
    P(route[0][nearindex],route[1][nearindex],(0,100,100),6)
    Ps(route[0],route[1],black,2)
    
    
    
    
    
    
    ##########
    mouse_x, mouse_y = pygame.mouse.get_pos()
    mpos=Pos(mouse_x,mouse_y)
    mouse_pos_text = font.render(f"robot: ({round(xc)}mm,{round(yc)}mm)", True, black)
    screen.blit(mouse_pos_text, (10, 10))  # 画面の左上に表示
    
    
    # 
    for I in click_positions:
         P(I[0],I[1],black,3)     
    
    
    # for posi in click_positions:
    #     pygame.draw.circle(screen, black, posi, 5) 
    
    for event in pygame.event.get():

        if event.type == pygame.QUIT:#やめるイベント
            running= False
            
          #キーボード  
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                UP=True               
            elif event.key == pygame.K_DOWN:
                DOWN=True
            elif event.key == pygame.K_RIGHT:
                RIGHT=True
            elif event.key == pygame.K_LEFT:
                LEFT=True
                
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                UP=False               
            elif event.key == pygame.K_DOWN:
                DOWN=False
            elif event.key == pygame.K_RIGHT:
                RIGHT=False
            elif event.key == pygame.K_LEFT:
                LEFT=False
                  
                
        #マウス        
        if event.type ==pygame.MOUSEBUTTONDOWN  :
            if event.button == 1:
                dragging= True
                pos=Pos(mouse_x,mouse_y)
                click_positions.append([round(pos[0]),round(pos[1])])
                
               
        if event.type ==pygame.MOUSEBUTTONUP :
            dragging = False
        #if event.type == pygame.MOUSEMOTION:
        
        if event.type == pygame.MOUSEWHEEL:
            if event.y > 0:  # 左クリック
                a *=1.1
                b *=1.1
            elif event.y <0:  # 右クリック
                a *=1/1.1
                b *=1/1.1
            
    if dragging:
        print("dragging")   
    if UP:oy+=10
    if DOWN:oy-=10
    if RIGHT:ox-=10
    if LEFT :ox+=10
            
    #print("load")       
    #時間とフレームの更新
    pygame.display.flip()
    clock.tick(10)
    
    
click_positions=np.array(click_positions) 

   
#np.save('spring25route3.npy',click_positions)
print(click_positions)

#print(field)
#終了    
pygame.quit()
sys.exit()
