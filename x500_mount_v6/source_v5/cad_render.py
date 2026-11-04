"""Software Z-buffer renderer of actual CAD triangles."""
import math
import numpy as np
from PIL import Image

def draw_cad(image,parts,origin,scale,explode=False,camera_direction=(1,-1,.78)):
    camera=np.array(camera_direction,dtype=float);camera/=np.linalg.norm(camera)
    right=np.cross(np.array([0.,0.,1.]),camera);right/=np.linalg.norm(right)
    up=np.cross(camera,right)
    light=np.array([-.2,-.5,1.]);light/=np.linalg.norm(light)
    def screen(v):return (origin[0]+scale*np.dot(v,right),origin[1]-scale*np.dot(v,up))
    pixels=np.array(image);zbuffer=np.full(pixels.shape[:2],-np.inf,dtype=float)
    for item in parts:
        name=item['name']
        if explode and name.startswith('REF_'):continue
        vertices=np.array(item['vertices'],dtype=float)
        if explode and 'collar' in name:
            vertices[:,1]+=(-45 if 'front' in name else 45)
            vertices[:,2]-=65
        for inds in item['faces']:
            tri=vertices[list(inds)];normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);n=np.linalg.norm(normal)
            if n<1e-10:continue
            normal/=n
            if np.dot(normal,camera)<-.001:continue
            shade=.60+.40*max(0,np.dot(normal,light))
            color=tuple(min(255,int(255*c*shade)) for c in item['color'])
            xy=np.array([screen(v) for v in tri]);depth=tri@camera
            xmin=max(0,int(math.floor(xy[:,0].min())));xmax=min(pixels.shape[1]-1,int(math.ceil(xy[:,0].max())))
            ymin=max(0,int(math.floor(xy[:,1].min())));ymax=min(pixels.shape[0]-1,int(math.ceil(xy[:,1].max())))
            if xmin>xmax or ymin>ymax:continue
            xx,yy=np.meshgrid(np.arange(xmin,xmax+1)+.5,np.arange(ymin,ymax+1)+.5)
            a,b,c=xy;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
            if abs(den)<1e-9:continue
            w0=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/den
            w1=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/den
            w2=1-w0-w1;zz=w0*depth[0]+w1*depth[1]+w2*depth[2]
            target=zbuffer[ymin:ymax+1,xmin:xmax+1]
            mask=(w0>=-1e-8)&(w1>=-1e-8)&(w2>=-1e-8)&(zz>target)
            target[mask]=zz[mask];pixels[ymin:ymax+1,xmin:xmax+1][mask]=color
    image.paste(Image.fromarray(pixels))
    return screen

