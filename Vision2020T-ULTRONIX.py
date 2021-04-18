import gym
import vision_arena
import time
import pybullet as p
import pybullet_data
import cv2
import cv2.aruco as aruco
import numpy as np



############### T S C
##black==0  red 1,2,3
##       yellow 4 5 6 


##up 1
## down 2
##right 3
## left 4
## ur 5
## dr 6
## lr 7
## dl 8
## ud 9
## lu 10
## url 11
## urd 12
## lrd 13
## uld 14

n=9
movementmat=np.zeros((9,9),np.uint8)
for i in range(n):
    movementmat[i][0]=1
    movementmat[0][i]=3
    movementmat[i][8]=2
    movementmat[8][i]=4


for i in range(2,7):
    movementmat[2][i]=3
    movementmat[6][i]=4

for i in range(2,7):
    movementmat[i][2]=1
    movementmat[i][6]=2


movementmat[4][1]=7
movementmat[4][2]=11
movementmat[1][4]=9
movementmat[2][4]=12
movementmat[4][7]=7
movementmat[4][6]=13
movementmat[7][4]=9
movementmat[6][4]=14

movementmat[2][2]=3
movementmat[6][6]=4
movementmat[0][8]=2
movementmat[4][0]=5
movementmat[0][4]=6
movementmat[4][8]=8
movementmat[8][4]=10



def stop():
    g=0
    while g<50:
        g+=1
        p.stepSimulation()
        env.move_husky(0.0,0.0,0.0,0.0)


def moveforward():
    cnt=0
    while cnt<=120:
        p.stepSimulation()
        if cnt%30==0:
            img=env.camera_feed()
        cnt+=1
        env.move_husky(4.3,4.3,4.3,4.3)
            #cv2.waitKey(5)

def moveright():
    cnt=0
    while(cnt<=110):
        if cnt%30==0:
            img=env.camera_feed()
        p.stepSimulation()
        env.move_husky(3.0,-3.0,3.0,-3.0)
        cnt+=1


def moveleft():
    cnt=0
    while(cnt<=110):
        if(cnt%30==0):
            img=env.camera_feed()
        p.stepSimulation()
        env.move_husky(-3.0,3.0,-3.0,3.0)
        cnt+=1

def predictshape(mask):
    kernel=np.ones((5,5),np.uint8)
    mask=cv2.erode(mask, kernel,iterations=1)
    mask=cv2.dilate(mask, kernel,iterations=1)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE, kernel)
    #cv2.imshow(str(k),mask)
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cntr in contours:
        area=cv2.contourArea(cntr)
        if area>=300:
            per=cv2.arcLength(cntr, closed=True)
            approx=cv2.approxPolyDP(cntr, epsilon=0.034*per, closed=True)
            #print(len(approx))
            if len(approx) is 3:
                return("triangle")
            elif len(approx) is 4:
                return("square")
            else:
                return("circle")


def start(corner):
    #corner=posdetect(img)
    #print(corner)
    y=(corner[0][0]+corner[2][0])//2
    x=(corner[0][1]+corner[2][1])//2
    #print(x,y)
    x=x//100
    y=y//100
    x=int(x)
    y=int(y)
    return([x,y])


def getcode(nxt):
    if nxt=="TR":
        return(1)
    elif nxt=="SR":
        return(2)
    elif nxt=="CR":
        return(3)
    elif nxt=="TY":
        return(4)
    elif nxt=="SY":
        return(5)
    else :
        return(6)


def unblockcode(corner):
    #corner=posdetect(initialimg)
    #print(corner)
    y=(corner[0][0]+corner[2][0])//2
    x=(corner[0][1]+corner[2][1])//2
    print(x,y)
    x=x//100
    y=y//100
    x=int(x)
    y=int(y)
    if(x==4 and y==0):
        return(2)#down
    elif(x==4 and y==8):
        return(1)#up
    elif(y==4 and x==0):
        return(4)#left
    else:
        return(3)#right

def lockrest(startingpos,colmat):
    col=-1
    lockedx=-1
    lockedy=-1
    if(startingpos[0]==4 and startingpos[1]==0):
        lockedx=4
        lockedy=3
        col=colmat[4][3]
        colmat[4][3]=-1
        colmat[3][4]=-1
        colmat[4][5]=-1
        colmat[5][4]=-1
    elif(startingpos[0]==0 and startingpos[1]==4):
        lockedx=3
        lockedy=4
        col=colmat[3][4]
        colmat[4][3]=-1
        colmat[3][4]=-1
        colmat[4][5]=-1
        colmat[5][4]=-1
    elif startingpos[0]==8 and startingpos[1]==4:
        lockedx=5
        lockedy=4
        col=colmat[5][4]
        colmat[4][3]=-1
        colmat[3][4]=-1
        colmat[4][5]=-1
        colmat[5][4]=-1
    else:
        lockedx=4
        lockedy=5
        col=colmat[4][5]
        colmat[4][3]=-1
        colmat[3][4]=-1
        colmat[4][5]=-1
        colmat[5][4]=-1
    return([col,lockedx,lockedy])



def movement(corner,dest):
    x=(corner[0][0]+corner[2][0])//2
    y=(corner[0][1]+corner[2][1])//2
    #print(x,y)
    vxreq=(dest[0]-x)
    vyreq=(dest[1]-y)
    modv=np.sqrt(vxreq**2+vyreq**2)
    vxreq=vxreq/modv
    vyreq=vyreq/modv
    botvx=(corner[0][0]-corner[3][0])
    botvy=(corner[0][1]-corner[3][1])
    mod=np.sqrt(botvx**2+botvy**2)
    botvx=botvx/mod
    botvy=botvy/mod
    botvec=complex(botvx,botvy)
    vec=complex(vxreq,vyreq)
    angle=np.angle(botvec/vec,deg=True)
    #print("angle :",angle)
    if(-4.4<angle<4.4):
        return("straight")
    elif(angle>=4.4):
        return("left")
    else:
        return("right")


def getpath(colmat,movementmat,img,destcode,unblock,locked,corner):
    global isover
    global islocked
    #corner=posdetect(img)
    #print(corner)
    y=(corner[0][0]+corner[2][0])//2
    x=(corner[0][1]+corner[2][1])//2
    #print(x,y)
    x=x//100
    y=y//100
    x=int(x)
    y=int(y)
    nmovementmat=np.zeros((9,9),np.uint8)
    nmovementmat=movementmat


    dist=np.zeros((n,n))
    inf=1000000000
    for i in range(n):
        for j in range(n):
            dist[i][j]=inf
    #print(nmovementmat)
    srcx=x
    srcy=y
    #print(srcx,srcy)
    par={}
    dist[srcx][srcy]=0
    destx=-1
    desty=-1
    q=[]
    q.append((srcx,srcy))
    lockedx=locked[1]
    lockedy=locked[2]
    while len(q)!=0:
        nodex,nodey=q.pop(0)
        

        if(lockedx==4 and lockedy==3):
            if nmovementmat[nodex][nodey]==3 or nmovementmat[nodex][nodey]==5 or nmovementmat[nodex][nodey]==6 or nmovementmat[nodex][nodey]==7 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==13:
                if(dist[nodex][nodey+1]==inf and colmat[nodex][nodey+1]>=0):
                    dist[nodex][nodey+1]=dist[nodex][nodey]+1
                    par[(nodex,nodey+1)]=(nodex,nodey)
                    q.append((nodex,nodey+1))
                    if(colmat[nodex][nodey+1]==destcode):
                        destx=nodex
                        desty=nodey+1
                        break
            if nmovementmat[nodex][nodey]==4 or nmovementmat[nodex][nodey]==7 or nmovementmat[nodex][nodey]==8 or nmovementmat[nodex][nodey]==10 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==13 or nmovementmat[nodex][nodey]==14:
                if(dist[nodex][nodey-1]==inf and colmat[nodex][nodey-1]>=0):
                    dist[nodex][nodey-1]=dist[nodex][nodey]+1
                    par[(nodex,nodey-1)]=(nodex,nodey)
                    q.append((nodex,nodey-1))
                    if(colmat[nodex][nodey-1]==destcode):
                        destx=nodex
                        desty=nodey-1
                        break
            if nmovementmat[nodex][nodey]==1 or nmovementmat[nodex][nodey]==5 or nmovementmat[nodex][nodey]==9 or nmovementmat[nodex][nodey]==10 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==14:
                if(dist[nodex-1][nodey]==inf and colmat[nodex-1][nodey]>=0):
                    dist[nodex-1][nodey]=dist[nodex][nodey]+1
                    par[(nodex-1,nodey)]=(nodex,nodey)
                    q.append((nodex-1,nodey))
                    if(colmat[nodex-1][nodey]==destcode):
                        destx=nodex-1
                        desty=nodey
                        break

            
            if nmovementmat[nodex][nodey]==2 or nmovementmat[nodex][nodey]==6 or nmovementmat[nodex][nodey]==8 or nmovementmat[nodex][nodey]==9 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==13 or nmovementmat[nodex][nodey]==14 :
                if(dist[nodex+1][nodey]==inf and colmat[nodex+1][nodey]>=0):
                    dist[nodex+1][nodey]=dist[nodex][nodey]+1
                    par[(nodex+1,nodey)]=(nodex,nodey)
                    q.append((nodex+1,nodey))
                    if(colmat[nodex+1][nodey]==destcode):
                        destx=nodex+1
                        desty=nodey
                        break

        elif(lockedx==4 and lockedy==5):
            if nmovementmat[nodex][nodey]==4 or nmovementmat[nodex][nodey]==7 or nmovementmat[nodex][nodey]==8 or nmovementmat[nodex][nodey]==10 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==13 or nmovementmat[nodex][nodey]==14:
                if(dist[nodex][nodey-1]==inf and colmat[nodex][nodey-1]>=0):
                    dist[nodex][nodey-1]=dist[nodex][nodey]+1
                    par[(nodex,nodey-1)]=(nodex,nodey)
                    q.append((nodex,nodey-1))
                    if(colmat[nodex][nodey-1]==destcode):
                        destx=nodex
                        desty=nodey-1
                        break
            if nmovementmat[nodex][nodey]==3 or nmovementmat[nodex][nodey]==5 or nmovementmat[nodex][nodey]==6 or nmovementmat[nodex][nodey]==7 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==13:
                if(dist[nodex][nodey+1]==inf and colmat[nodex][nodey+1]>=0):
                    dist[nodex][nodey+1]=dist[nodex][nodey]+1
                    par[(nodex,nodey+1)]=(nodex,nodey)
                    q.append((nodex,nodey+1))
                    if(colmat[nodex][nodey+1]==destcode):
                        destx=nodex
                        desty=nodey+1
                        break
            if nmovementmat[nodex][nodey]==1 or nmovementmat[nodex][nodey]==5 or nmovementmat[nodex][nodey]==9 or nmovementmat[nodex][nodey]==10 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==14:
                if(dist[nodex-1][nodey]==inf and colmat[nodex-1][nodey]>=0):
                    dist[nodex-1][nodey]=dist[nodex][nodey]+1
                    par[(nodex-1,nodey)]=(nodex,nodey)
                    q.append((nodex-1,nodey))
                    if(colmat[nodex-1][nodey]==destcode):
                        destx=nodex-1
                        desty=nodey
                        break

            
            if nmovementmat[nodex][nodey]==2 or nmovementmat[nodex][nodey]==6 or nmovementmat[nodex][nodey]==8 or nmovementmat[nodex][nodey]==9 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==13 or nmovementmat[nodex][nodey]==14 :
                if(dist[nodex+1][nodey]==inf and colmat[nodex+1][nodey]>=0):
                    dist[nodex+1][nodey]=dist[nodex][nodey]+1
                    par[(nodex+1,nodey)]=(nodex,nodey)
                    q.append((nodex+1,nodey))
                    if(colmat[nodex+1][nodey]==destcode):
                        destx=nodex+1
                        desty=nodey
                        break

        elif(lockedx==3 and lockedy==4):
            if nmovementmat[nodex][nodey]==4 or nmovementmat[nodex][nodey]==7 or nmovementmat[nodex][nodey]==8 or nmovementmat[nodex][nodey]==10 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==13 or nmovementmat[nodex][nodey]==14:
                if(dist[nodex][nodey-1]==inf and colmat[nodex][nodey-1]>=0):
                    dist[nodex][nodey-1]=dist[nodex][nodey]+1
                    par[(nodex,nodey-1)]=(nodex,nodey)
                    q.append((nodex,nodey-1))
                    if(colmat[nodex][nodey-1]==destcode):
                        destx=nodex
                        desty=nodey-1
                        break
            if nmovementmat[nodex][nodey]==1 or nmovementmat[nodex][nodey]==5 or nmovementmat[nodex][nodey]==9 or nmovementmat[nodex][nodey]==10 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==14:
                if(dist[nodex-1][nodey]==inf and colmat[nodex-1][nodey]>=0):
                    dist[nodex-1][nodey]=dist[nodex][nodey]+1
                    par[(nodex-1,nodey)]=(nodex,nodey)
                    q.append((nodex-1,nodey))
                    if(colmat[nodex-1][nodey]==destcode):
                        destx=nodex-1
                        desty=nodey
                        break
            if nmovementmat[nodex][nodey]==3 or nmovementmat[nodex][nodey]==5 or nmovementmat[nodex][nodey]==6 or nmovementmat[nodex][nodey]==7 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==13:
                if(dist[nodex][nodey+1]==inf and colmat[nodex][nodey+1]>=0):
                    dist[nodex][nodey+1]=dist[nodex][nodey]+1
                    par[(nodex,nodey+1)]=(nodex,nodey)
                    q.append((nodex,nodey+1))
                    if(colmat[nodex][nodey+1]==destcode):
                        destx=nodex
                        desty=nodey+1
                        break
            if nmovementmat[nodex][nodey]==2 or nmovementmat[nodex][nodey]==6 or nmovementmat[nodex][nodey]==8 or nmovementmat[nodex][nodey]==9 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==13 or nmovementmat[nodex][nodey]==14 :
                if(dist[nodex+1][nodey]==inf and colmat[nodex+1][nodey]>=0):
                    dist[nodex+1][nodey]=dist[nodex][nodey]+1
                    par[(nodex+1,nodey)]=(nodex,nodey)
                    q.append((nodex+1,nodey))
                    if(colmat[nodex+1][nodey]==destcode):
                        destx=nodex+1
                        desty=nodey
                        break

        else:
            if nmovementmat[nodex][nodey]==1 or nmovementmat[nodex][nodey]==5 or nmovementmat[nodex][nodey]==9 or nmovementmat[nodex][nodey]==10 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==14:
                if(dist[nodex-1][nodey]==inf and colmat[nodex-1][nodey]>=0):
                    dist[nodex-1][nodey]=dist[nodex][nodey]+1
                    par[(nodex-1,nodey)]=(nodex,nodey)
                    q.append((nodex-1,nodey))
                    if(colmat[nodex-1][nodey]==destcode):
                        destx=nodex-1
                        desty=nodey
                        break
            if nmovementmat[nodex][nodey]==2 or nmovementmat[nodex][nodey]==6 or nmovementmat[nodex][nodey]==8 or nmovementmat[nodex][nodey]==9 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==13 or nmovementmat[nodex][nodey]==14 :
                if(dist[nodex+1][nodey]==inf and colmat[nodex+1][nodey]>=0):
                    dist[nodex+1][nodey]=dist[nodex][nodey]+1
                    par[(nodex+1,nodey)]=(nodex,nodey)
                    q.append((nodex+1,nodey))
                    if(colmat[nodex+1][nodey]==destcode):
                        destx=nodex+1
                        desty=nodey
                        break
            if nmovementmat[nodex][nodey]==4 or nmovementmat[nodex][nodey]==7 or nmovementmat[nodex][nodey]==8 or nmovementmat[nodex][nodey]==10 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==13 or nmovementmat[nodex][nodey]==14:
                if(dist[nodex][nodey-1]==inf and colmat[nodex][nodey-1]>=0):
                    dist[nodex][nodey-1]=dist[nodex][nodey]+1
                    par[(nodex,nodey-1)]=(nodex,nodey)
                    q.append((nodex,nodey-1))
                    if(colmat[nodex][nodey-1]==destcode):
                        destx=nodex
                        desty=nodey-1
                        break
            if nmovementmat[nodex][nodey]==3 or nmovementmat[nodex][nodey]==5 or nmovementmat[nodex][nodey]==6 or nmovementmat[nodex][nodey]==7 or nmovementmat[nodex][nodey]==11 or nmovementmat[nodex][nodey]==12 or nmovementmat[nodex][nodey]==13:
                if(dist[nodex][nodey+1]==inf and colmat[nodex][nodey+1]>=0):
                    dist[nodex][nodey+1]=dist[nodex][nodey]+1
                    par[(nodex,nodey+1)]=(nodex,nodey)
                    q.append((nodex,nodey+1))
                    if(colmat[nodex][nodey+1]==destcode):
                        destx=nodex
                        desty=nodey+1
                        break
    l=[]
    if(destx==locked[1] and desty==locked[2]):
        isover=True
    #print(isover,"isover")
    #print(islocked,"islocked")
    #print(destx,desty)
    l.append((destx,desty))
    while destx!=srcx or desty!=srcy:
        destx,desty=par[(destx,desty)]
        l.append((destx,desty))
    l.reverse()
    path=np.array(l)
    t=False
    if(islocked==False):
        for i in range(len(path)):
            #pathfin.append(path[i])
            if(i!=len(path)-1):
                if(locked[1]==4 and locked[2]==3):
                    if(path[i+1][0]==3 and (path[i+1][1]==0 or path[i+1][1]==2)):
                        t=True
                        break
                if(locked[1]==4 and locked[2]==5):
                    if(path[i+1][0]==5 and (path[i+1][1]==8 or path[i+1][1]==6)):
                        t=True
                        break
                if(locked[1]==3 and locked[2]==4):
                    if(path[i+1][1]==5 and (path[i+1][0]==0 or path[i+1][0]==2)):
                        t=True
                        break 
                if(locked[1]==5 and locked[2]==4):
                    if(path[i+1][1]==3 and (path[i+1][0]==8 or path[i+1][0]==6)):
                        t=True
                        break     
    print(path)
    if(t==True):
        #print("verynear1")
        return([])


    for i in range(len(path)):
        print(path[i][0],path[i][1],end=" ")
        if(i!=len(path)-1):
            if(path[i+1][1]-path[i][1]==1):
                #right
                if(unblock==3):
                    islocked=False
                    print("y1")
            if(path[i+1][0]-path[i][0]==1):
                #down 
                if(unblock==2):
                    islocked=False
                    print("y2")
            if(path[i+1][0]-path[i][0]==-1):
                #up
                if(unblock==1):
                    islocked=False
                    print("y3")
            if(path[i+1][1]-path[i][1]==-1):
                #left
                if(unblock==4):
                    islocked=False
                    print("y4")
            if(islocked==False and locked[1]==4 and locked[2]==3):
                if(path[i+1][0]==3 and (path[i+1][1]==0 or path[i+1][1]==2)):
                    t=True
                    print("y5")
                    break
            if(islocked==False and locked[1]==4 and locked[2]==5):
                if(path[i+1][0]==5 and (path[i+1][1]==8 or path[i+1][1]==6)):
                    t=True
                    print("y6")
                    break
            if(islocked==False and locked[1]==3 and locked[2]==4):
                if(path[i+1][1]==5 and (path[i+1][0]==0 or path[i+1][0]==2)):
                    t=True
                    print("y7")
                    break 
            if(islocked==False and locked[1]==5 and locked[2]==4):
                if(path[i+1][1]==3 and (path[i+1][0]==8 or path[i+1][0]==6)):
                    t=True
                    break
            #img=cv2.arrowedLine(img, pt1=(path[i][1]*100+50,path[i][0]*100+50), pt2=(path[i+1][1]*100+50,path[i+1][0]*100+50), color=(0,255,0) ,thickness=5)
            print("--->",end=" ")


    if islocked==False:
        colmat[locked[1]][locked[2]]=locked[0]

    if(t==True):
        #print("very near2")
        return([])

    return(path)


def posdetect(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    #img = aruco.drawDetectedMarkers(img, corners, borderColor=(0, 0, 255))
    if ids is None:
        return([])
    corners=np.array(corners)
    corner=corners[0]
    corner=corner.reshape((corner.shape[1],corner.shape[2]))
    return(corner)

def botvec(corner):
    botvx=(corner[0][0]-corner[3][0])
    botvy=(corner[0][1]-corner[3][1])
    mod=np.sqrt(botvx**2+botvy**2)
    botvx=botvx/mod
    botvy=botvy/mod
    return([botvx,botvy])


def predictcolor(box):
    x,y,z=box[50,50]
    #print(x,y,z)
    if z<=20:
        return(-1)
    elif z<=160:
        #red
        lowerb=np.array([0,0,84])
        upperb=np.array([42,128,255])
        mask=cv2.inRange(box, lowerb, upperb)
        shape=predictshape(mask)
        if(shape=="triangle"):
            return(1)
        elif shape=="square":
            return(2)
        else:
            return(3)   
    else:
        #yellow
        lowerb=np.array([0,0,185])
        upperb=np.array([76,255,255])
        mask=cv2.inRange(box, lowerb, upperb)
        shape=predictshape(mask)
        if(shape=="triangle"):
            return(4)
        elif shape=="square":
            return(5)
        else:
            return(6)


def colormatrix(img):
    colmat=np.zeros((n,n))
    for i in range(n):
        for j in range(n):
            if(i==4 and j==0):
                colmat[i][j]=0
                continue##arrow
            if(i==0 and j==4):
                colmat[i][j]=0
                continue
            if(i==8 and j==4):
                colmat[i][j]=0
                continue
            if(j==8 and i==4):
                colmat[i][j]=0
                continue
            if(i==4 and j==4):
                colmat[i][j]=0
                continue
            ##print(i,j)
            c=predictcolor(img[i*100:(i+1)*100,j*100:(j+1)*100])
            ##cv2.imshow(str(k),img[i*100:(i+1)*100,j*100:(j+1)*100])
            colmat[i][j]=c
            #print(colmat[i][j],end=" ")
        #print(" ")
    colmat[4][4]=10
    return colmat



def imgcrop(img,r):
    img=img[int(r[1]):int(r[1]+r[3]),int(r[0]):int(r[0]+r[2])]
    img=cv2.resize(img,(900,900))
    return(img)


def dist(img,dest):
    corner=posdetect(img)
    if(len(corner)==0):
        return -1
    y=(corner[0][0]+corner[2][0])//2
    x=(corner[0][1]+corner[2][1])//2
    return(np.sqrt((x-dest[1])**2+(y-dest[0])**2))

if __name__=="__main__":
    env = gym.make("vision_arena-v0")
    env.remove_car()
    img =env.camera_feed()
    r=cv2.selectROI(img)
    img=imgcrop(img,r)
    colmat=colormatrix(img)
    #img =env.camera_feed()
    #r=cv2.selectROI(img)
    env.respawn_car()
    corner=[]
    p.stepSimulation()
    y=0
    while(len(corner)==0):
        img = env.camera_feed()
        #img=imgcrop(img,r)
        img=cv2.resize(img,(900,900))
        corner=posdetect(img)
        env.move_husky(1,-1,1,-1)
    startingpos=start(corner)
    #print(startingpos)
    unblock=unblockcode(corner)
    #print(unblock)
    locked=lockrest(startingpos, colmat)
    #print(locked)
    #print(colmat)
    isover=False
    islocked=True
    while isover==False:
        img = env.camera_feed()
            #img=imgcrop(img,r)
        img=cv2.resize(img,(900,900))
        corner=posdetect(img)
        #env.move_husky(7.0,-7.0,7.0,-7.0)
        if(len(corner)==0):
            continue
        nxtmove=env.roll_dice()
        print(nxtmove)
        nxt=getcode(nxtmove)
        print(nxt)
        path=getpath(colmat, movementmat, img , nxt, unblock, locked,corner)
        print(path)
        if(len(path)<=1):
            print("stoppped")
            p.stepSimulation()
            env.move_husky(0,0,0,0)
            p.stepSimulation()
            continue
        ##print(isover)
        #print(islocked)
        #print(colmat)
        k=1
        while k<len(path):
            print(path[k][0],path[k][1])
            dest=[path[k][1]*100+53,path[k][0]*100+52]
            while True:
                img=env.camera_feed()
                img=cv2.resize(img,(900,900))
                distance=dist(img,dest)
                #print(distance)
                if(distance==-1):
                    moveright()
                    stop()
                    continue
                if(distance<20.0):
                    break
                corner=posdetect(img)
                move=movement(corner, dest)
                #print(move)
                if(move=="right"):
                    moveright()
                    stop()
                elif(move=="left"):
                    moveleft()
                    stop()
                else:
                    moveforward()
                    stop()

            print("reached dest")
            p.stepSimulation()
            stop()
            k+=1
    dest=[4*100+53,4*100+52]
    while True:
        img=env.camera_feed()
        img=cv2.resize(img,(900,900))
        distance=dist(img,dest)
                #print(distance)
        if(distance==-1):
            moveright()
            stop()
            continue
        if(distance<20.0):
            break
        corner=posdetect(img)
        move=movement(corner, dest)
            #print(move)
        if(move=="right"):
            moveright()
            stop()
        elif(move=="left"):
            moveleft()
            stop()
        else:
            moveforward()
            stop()

    print("GAME OVER")
    time.sleep(100)
