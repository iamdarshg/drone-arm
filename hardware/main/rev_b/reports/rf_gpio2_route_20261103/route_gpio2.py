import pcbnew as p, math, heapq, numpy as np, pathlib, hashlib, sys, os
HERE=pathlib.Path(__file__).resolve().parent
src=HERE/'prep.kicad_pcb'
out=HERE/'replayed_candidate.kicad_pcb'
expected='19f3405fee98c1da4a156c6db314f16d2fac410a064ccc927a9e7ce96b43890d'
assert hashlib.sha256(src.read_bytes()).hexdigest()==expected
b=p.LoadBoard(str(src)); net=b.FindNet('/RF_GPIO2').GetNetCode(); mm=p.FromMM; V=lambda q:p.VECTOR2I(round(q[0]*1e6),round(q[1]*1e6))
def segdist(x,y,a,z):
 dx=z[0]-a[0];dy=z[1]-a[1];d=dx*dx+dy*dy;u=0 if not d else max(0,min(1,((x-a[0])*dx+(y-a[1])*dy)/d));return math.hypot(x-a[0]-u*dx,y-a[1]-u*dy)
def route_astar(layer,start,goal,step=.1,bounds=(50,48,110,68),width=.2,extra_clear=.125):
 xmin,ymin,xmax,ymax=bounds; nx=round((xmax-xmin)/step)+1; ny=round((ymax-ymin)/step)+1; occ=np.zeros((nx,ny),dtype=np.bool_)
 def mark(x0,y0,x1,y1,fn):
  ilo=max(0,int((x0-xmin)/step)-1);ihi=min(nx-1,int((x1-xmin)/step)+1);jlo=max(0,int((y0-ymin)/step)-1);jhi=min(ny-1,int((y1-ymin)/step)+1)
  for i in range(ilo,ihi+1):
   for j in range(jlo,jhi+1):
    x=xmin+i*step;y=ymin+j*step
    if fn(x,y):occ[i,j]=True
 radius=width/2+extra_clear
 for t in b.GetTracks():
  if t.GetNetname()=='/RF_GPIO2':continue
  if isinstance(t,p.PCB_VIA):
   if t.GetLayerSet().Contains(layer):
    q=t.GetPosition();x=q.x/1e6;y=q.y/1e6;r=t.GetWidth(layer)/2e6+radius
    mark(x-r,y-r,x+r,y+r,lambda a,z,x=x,y=y,r=r:(a-x)**2+(z-y)**2<=r*r)
  elif t.IsOnLayer(layer):
   a=(t.GetStart().x/1e6,t.GetStart().y/1e6);z=(t.GetEnd().x/1e6,t.GetEnd().y/1e6);r=radius+t.GetWidth()/2e6
   mark(min(a[0],z[0])-r,min(a[1],z[1])-r,max(a[0],z[0])+r,max(a[1],z[1])+r,lambda x,y,a=a,z=z,r=r:segdist(x,y,a,z)<=r)
 for fp in b.GetFootprints():
  for pad in fp.Pads():
   if pad.GetNetname()=='/RF_GPIO2' or not pad.IsOnLayer(layer):continue
   x=pad.GetPosition().x/1e6;y=pad.GetPosition().y/1e6;hx=pad.GetSize().x/2e6;hy=pad.GetSize().y/2e6;ang=math.radians(pad.GetOrientationDegrees());c=math.cos(ang);s=math.sin(ang);r=radius
   bx=abs(c)*hx+abs(s)*hy+r;by=abs(s)*hx+abs(c)*hy+r
   mark(x-bx,y-by,x+bx,y+by,lambda a,z,x=x,y=y,hx=hx,hy=hy,c=c,s=s,r=r:(max(abs((a-x)*c+(z-y)*s)-hx,0)**2+max(abs(-(a-x)*s+(z-y)*c)-hy,0)**2)<r*r)
 si=(round((start[0]-xmin)/step),round((start[1]-ymin)/step));gi=(round((goal[0]-xmin)/step),round((goal[1]-ymin)/step));print(layer,'blocked endpoints',occ[si],occ[gi],flush=True);occ[si]=occ[gi]=False
 moves=[(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,2**.5),(1,-1,2**.5),(-1,1,2**.5),(-1,-1,2**.5)];q=[(math.dist(si,gi),0,si)];d={si:0};pr={};closed=set()
 while q:
  _,cost,u=heapq.heappop(q)
  if u in closed:continue
  if u==gi:break
  closed.add(u)
  for dx,dy,w in moves:
   v=(u[0]+dx,u[1]+dy)
   if not(0<=v[0]<nx and 0<=v[1]<ny) or occ[v] or (dx and dy and (occ[u[0]+dx,u[1]] or occ[u[0],u[1]+dy])):continue
   nc=cost+w
   if nc<d.get(v,1e99):d[v]=nc;pr[v]=u;heapq.heappush(q,(nc+math.dist(v,gi),nc,v))
 else:raise RuntimeError(f'NO_PATH layer {layer}, explored {len(closed)}')
 path=[gi];u=gi
 while u!=si:u=pr[u];path.append(u)
 path.reverse();pts=[(xmin+i*step,ymin+j*step) for i,j in path];simp=[pts[0]]
 for k in range(1,len(pts)-1):
  a=simp[-1];x=pts[k];z=pts[k+1]
  if abs((x[0]-a[0])*(z[1]-x[1])-(x[1]-a[1])*(z[0]-x[0]))>1e-8:simp.append(x)
 simp.append(pts[-1]);simp[0]=start;simp[-1]=goal;print(layer,'path',simp,flush=True);return simp
def addroute(points,layer,width=.2):
 for a,z in zip(points,points[1:]):
  t=p.PCB_TRACK(b);t.SetStart(V(a));t.SetEnd(V(z));t.SetLayer(layer);t.SetWidth(mm(width));t.SetNetCode(net);b.Add(t)
def addvia(pos,kind,pair,dia=.3,drill=.1):
 v=p.PCB_VIA(b);v.SetPosition(V(pos));v.SetNetCode(net);v.SetViaType(kind);v.SetLayerPair(*pair);v.SetDrill(mm(drill));
 for L in pair:v.SetWidth(L,mm(dia))
 b.Add(v)
# U10 pad breakout and first span (source via positions from previously DRC-clean topology).
addroute([(55.6,54.95),(55.6,55.2)],p.F_Cu)
addvia((55.6,55.2),p.VIATYPE_MICROVIA,(p.F_Cu,p.In2_Cu))
addvia((58.0,58.1),p.VIATYPE_BLIND_BURIED,(p.In2_Cu,p.B_Cu),.6,.3)
addroute(route_astar(p.In2_Cu,(55.6,55.2),(58.0,58.1),bounds=(50,48,70,68),width=.2),p.In2_Cu)
# Route B north/outer and In1 final approach.
bpath=route_astar(p.B_Cu,(58.0,58.1),(101,56),bounds=(45,45,115,96),width=.2)
addroute(bpath,p.B_Cu)
addvia((101,56),p.VIATYPE_THROUGH,(p.F_Cu,p.B_Cu),.6,.3)
# Internal route to F/In1 microvia in target pad.
addvia((91.5625,60.75),p.VIATYPE_MICROVIA,(p.F_Cu,p.In1_Cu))
ipath=route_astar(p.In1_Cu,(101,56),(91.5625,60.75),bounds=(88,54,104,67),width=.2)
addroute(ipath,p.In1_Cu)
p.ZONE_FILLER(b).Fill(b.Zones());p.SaveBoard(str(out),b);print('saved',out,hashlib.sha256(out.read_bytes()).hexdigest());sys.stdout.flush();os._exit(0)
