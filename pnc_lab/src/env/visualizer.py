import matplotlib.pyplot as plt
import math

class Visualizer:
    def __init__(self, config):
        self.m=config['map']; self.v=config['vehicle']; self.show=config.get('vis',{}).get('show_obstacles',False); self.p=config.get('vis',{}).get('pause',0.001); self.obs=[]
        try:
            plt.ion(); self.fig,(self.ax,self.ax1,self.ax2)=plt.subplots(1,3,figsize=(15,5))
            self.h1=[]; self.h2=[]
        except: pass

    def render(self, path, s, acc=0.0, steer=0.0):
        if not plt.get_fignums(): return
        self.ax.cla(); self.ax.set_aspect('equal')
        self.ax.set_xlim(0,self.m['width']); self.ax.set_ylim(0,self.m['height'])
        if self.show:
            [self.ax.add_patch(plt.Circle((o[1],o[2]),o[3],fc='k')) if o[0]=='c' else self.ax.add_patch(plt.Rectangle((o[1]-o[3]/2,o[2]-o[4]/2),o[3],o[4],fc='k')) for o in self.obs]
        if path: self.ax.plot(*zip(*[(p.x,p.y) for p in path]),'r--')
        hl=self.v['car_length']/2; hw=self.v['car_width']/2; c=math.cos(s.yaw); d=math.sin(s.yaw)
        pts=[(hl,hw),(hl,-hw),(-hl,-hw),(-hl,hw),(hl,hw)]; R=lambda X,Y:(s.x+X*c-Y*d,s.y+X*d+Y*c)
        poly=[R(*p) for p in pts]; self.ax.add_patch(plt.Polygon(poly,fc='b',ec='b',alpha=.6))
        for X,Y in [(hl,hw),(hl,-hw),(-hl,-hw),(-hl,hw)]:
            wx,wy=R(X,Y); self.ax.add_patch(plt.Circle((wx,wy),0.2,fc='k'))
        self.h1.append(acc); self.h2.append(steer)
        self.ax1.cla(); self.ax1.plot(self.h1,'g'); self.ax1.set_title('Lon Acc Cmd'); self.ax1.grid(True)
        self.ax2.cla(); self.ax2.plot(self.h2,'m'); self.ax2.set_title('Lat Steer Cmd'); self.ax2.grid(True)
        plt.pause(self.p)

    def close(self):
        try: plt.ioff(); plt.close('all')
        except: pass
