import numpy as np
import matplotlib.pyplot as plt
from operator import add
import sympy as sy
from sympy.polys.polyfuncs import interpolate



save_simulation = False 
def A_Star(obs,start,end):
    start_node = [start]
    end_node=[end]
    f=[np.hypot(start_node[0][0]-end_node[0][0],start_node[0][1]-end_node[0][1])] # Total cost
    g=[0] # Distance
    h=[f[0]] #huerestic cost
    imgct=0

    open_list = [start_node[0]]
    closed_list=[]

    children =[]
    traverse=[[0,1],[1,1],[1,0],[-1,0],[0,-1],[-1,-1],[-1,1],[1,-1]]
    distance_cost=[1.0,1.4,1.0,1.0,1.0,1.4,1.4,1.4]

    obstacle =obs
    
    
    obstacle1=np.asarray(obstacle)
    plt.plot(obstacle1[:,0],obstacle1[:,1],'x')
    parent=[]
    child_1=[]
    
    
    
  

    while open_list:
    
        f=list( map(add, g, h) )
        f_min= min(f)
        index=f.index(f_min)
       
        current_node=open_list.pop(index)
        closed_list.append(current_node)
        
        current_node_g=g[index]
        f.pop(index)
        g.pop(index)
        h.pop(index)
        print(1)
      
        if current_node==end_node[0]:
            print("Reached")
           
          
            path=[]
            leng=len(closed_list)
            current=closed_list[leng-1]
           
           
            while current is not start_node[0]:
               
               
                ch=child_1.index(current)
                path.append(parent[ch])
              
               
                if parent[ch] != start_node[0]:
                    ind=child_1.index(parent[ch])
                    current=child_1[ind]
                    

                else:
                    break
            return path
            
    
        children=[]
        children_g=[]
        children_h=[]
        children_f=[]
        

        for i in range(8):
            
            if list( map(add, current_node, traverse[i] )):
                children.append(list( map(add, current_node, traverse[i] ))) 
                children_h.append(np.hypot(children[i][0]-end_node[0][0],children[i][1]-end_node[0][1]))
                children_f.append(np.hypot(children[i][0]-end_node[0][0],children[i][1]-end_node[0][1]) + distance_cost[i]+current_node_g)
                children_g.append(distance_cost[i]+current_node_g)
           

        for k,j in enumerate(children):
        
          
            
            if j in obstacle:
                continue
           
            if j in closed_list:    
                continue
            
            if j not in open_list:
                open_list.append(j)
                parent.append(current_node)
                child_1.append(j)
                g.append(children_g[k])
                h.append(children_h[k])
                f.append(children_f[k])
               
          
            
            for ind,open_node in enumerate(open_list):
                if j==open_node and children_g[k]<=g[ind]:
                    for ind1,open_node1 in enumerate(child_1):
                        if open_node1==j:    
                            parent[ind1]=current_node
                      
                    g[ind]=children_g[k]
                    h[ind]=children_h[k]
                    f[ind]=children_f[k]
                  
            

            imgct+=1
            if save_simulation and imgct%50==0:
                cl=np.asarray(closed_list)           
                plt.plot(cl[:,0],cl[:,1],'.y')
                op=np.asarray(open_list)
                plt.plot(op[:,0],op[:,1],'.g')
                plt.pause(0.0001)
                plt.savefig('astar_'+str(imgct))
               


def path(pts,startX,stopX):
    
    x=sy.Symbol('x')
    y= sy.interpolate(pts,x)
    x=np.linspace(startX,stopX,100)
    def f(x):
        return y
    
    return f(x)
  

def design():
    pts=[(1,-20),(5,-20),(100,-20)]
    startX=0
    stopX=100
    p1=path(pts,startX,stopX)
    x=np.linspace(startX,stopX,100)
    p1=p1*x/x
    #plt.plot(x,p1)
    road=[]
    for i in range(51):
        road.append([i,45])
        road.append([i,5])
     
    for i in range(-5,56):
        road.append([i,0])
        road.append([i,50])
     
  
    for i in range(5,45):
        road.append([50,i])
        road.append([0,i])
    
    for i in range(0,50):
        road.append([55,i])
        road.append([-5,i])
    return road
    

        
def get_astar_path(sx,sy,ex,ey):
    
    obs=design()
    plt.plot(ex,ey,'xm')
    plt.plot(sx,sy,'xm')
    path = A_Star(obs,[sx,sy],[ex, ey])
    path.reverse()
    
    nppath=np.asarray(path)    
    print(path)       
    plt.plot(nppath[:,0],nppath[:,1],'xr')
    plt.show()   

    return path

if __name__ == '__main__':
    get_astar_path(sx=10,sy=2,ex=35,ey=47)  
