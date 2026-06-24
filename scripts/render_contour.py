#!/usr/bin/env python3
"""Top-down HEIGHT map (contour) from an RTAB-Map PLY cloud. Colors = height (Z),
black iso-lines every CONTOUR_M metres. Pure numpy + stdlib PNG (no matplotlib)."""
import sys, struct, zlib, numpy as np

ply_path, png_path = sys.argv[1], sys.argv[2]
CELL = 0.05            # grid cell size (m)
CONTOUR_M = 0.10       # contour interval (m) — height above ground
UP = 3                 # upscale factor (px per cell)

# ---- parse binary PLY vertices (x y z f4, r g b u1, nx ny nz curv f4 = 31 bytes) ----
raw = open(ply_path, "rb").read()
he = raw.index(b"end_header\n") + len(b"end_header\n")
hdr = raw[:he].decode("ascii", "replace")
n = next(int(l.split()[2]) for l in hdr.splitlines() if l.startswith("element vertex"))
dt = np.dtype([("x","<f4"),("y","<f4"),("z","<f4"),("r","u1"),("g","u1"),("b","u1"),
               ("nx","<f4"),("ny","<f4"),("nz","<f4"),("c","<f4")])
v = np.frombuffer(raw[he:he+n*dt.itemsize], dtype=dt, count=n)
x, y, z = v["x"].astype(np.float64), v["y"].astype(np.float64), v["z"].astype(np.float64)

# ---- height grid: max Z per XY cell (top surface seen from above) ----
xmin, ymin = x.min(), y.min()
gw = int((x.max()-xmin)/CELL) + 1
gh = int((y.max()-ymin)/CELL) + 1
H = np.full((gh, gw), np.nan)
ix = ((x-xmin)/CELL).astype(np.int32)
iy = ((y.max()-y)/CELL).astype(np.int32)        # flip: +y up
order = np.argsort(z)                             # highest z wins per cell
H[iy[order], ix[order]] = z[order]

# ---- fill small gaps so the surface reads as continuous (a few neighbour passes) ----
for _ in range(4):
    nan = np.isnan(H)
    if not nan.any(): break
    s = np.zeros_like(H); cnt = np.zeros_like(H)
    for dy in (-1,0,1):
        for dx in (-1,0,1):
            if dx==0 and dy==0: continue
            sh = np.roll(np.roll(np.where(nan,0.0,H),dy,0),dx,1)
            m  = np.roll(np.roll((~nan).astype(float),dy,0),dx,1)
            s += sh; cnt += m
    fill = nan & (cnt>=3)
    H[fill] = (s/np.maximum(cnt,1))[fill]

# ---- detrend: the SLAM ground plane is tilted (odom drift), so fit a plane to the
#      ground and measure HEIGHT ABOVE GROUND. Objects then stand out, ground ~ 0. ----
valid = ~np.isnan(H)
yy, xx = np.nonzero(valid)
Xm = xmin + xx*CELL; Ym = ymin + yy*CELL          # cell centres in metres
Hv = H[valid]
# robust plane fit: least squares, then refit on the lower (ground) residuals only
A = np.column_stack([Xm, Ym, np.ones_like(Xm)])
coef, *_ = np.linalg.lstsq(A, Hv, rcond=None)
res = Hv - A @ coef
ground = res < np.percentile(res, 70)             # drop the object-top tail
coef, *_ = np.linalg.lstsq(A[ground], Hv[ground], rcond=None)
Hdet = np.full_like(H, np.nan)
Hdet[valid] = Hv - A @ coef                        # height above fitted ground
H = Hdet                                            # contour lines use the detrended field too

zlo, zhi = 0.0, np.percentile(H[valid], 99)        # ground=0 -> object tops
norm = np.clip((H - zlo)/max(zhi-zlo, 1e-6), 0, 1)

# ---- elevation colormap: blue(low)->cyan->green->yellow->red(high) via HSV hue 240->0 ----
def hsv(hue):                                    # hue 0..1 (=240deg..0deg), s=v=1
    h6 = hue*6.0; i = np.floor(h6).astype(int) % 6; f = h6-np.floor(h6)
    p = np.zeros_like(hue); q = 1-f; t = f
    r = np.choose(i,[1,q,p,p,t,1]); g = np.choose(i,[t,1,1,q,p,p]); b = np.choose(i,[p,p,t,1,1,q])
    return r,g,b
hue = np.where(valid, (1-norm)*(240/360.0), 0.0) # high -> red(0), low -> blue(240)
r,g,b = hsv(hue)
img = np.stack([r,g,b],-1)
img[~valid] = 1.0                                # no-data = white

# ---- contour iso-lines: band changes between neighbours (marching-squares-lite) ----
band = np.where(valid, np.round(H/CONTOUR_M), np.nan)
edge = np.zeros((gh,gw), bool)
for dy,dx in ((0,1),(1,0)):
    diff = (band != np.roll(np.roll(band,-dy,0),-dx,1)) & valid & \
           np.roll(np.roll(valid,-dy,0),-dx,1)
    edge |= diff
img[edge] = 0.0                                  # black contour lines

# ---- upscale (nearest) for a crisp image ----
img = np.repeat(np.repeat(img, UP, 0), UP, 1)
arr = (img*255).astype(np.uint8)

# ---- colorbar strip on the right (no text; range printed below) ----
H2,W2,_ = arr.shape
bar_w = 26
cb = np.zeros((H2,bar_w,3),np.uint8)
gy = np.linspace(1,0,H2)                          # top=high
br,bg,bb = hsv((1-gy)*(240/360.0))
cb[:,:,0]=(br*255)[:,None]; cb[:,:,1]=(bg*255)[:,None]; cb[:,:,2]=(bb*255)[:,None]
sep = np.full((H2,4,3),255,np.uint8)
arr = np.concatenate([arr, sep, cb], 1)

# ---- pure-stdlib PNG ----
def write_png(path, a):
    h,w,_ = a.shape
    rows = b"".join(b"\x00"+a[i].tobytes() for i in range(h))
    def ch(t,d): return struct.pack(">I",len(d))+t+d+struct.pack(">I",zlib.crc32(t+d)&0xffffffff)
    with open(path,"wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(ch(b"IHDR",struct.pack(">IIBBBBB",w,h,8,2,0,0,0)))
        f.write(ch(b"IDAT",zlib.compress(rows,9))); f.write(ch(b"IEND",b""))
write_png(png_path, arr)

print(f"grid {gw}x{gh} cells @ {CELL} m  | contour every {CONTOUR_M} m")
print(f"height range (2-98 pct): {zlo:.2f} .. {zhi:.2f} m  (full {H[valid].min():.2f}..{H[valid].max():.2f})")
print(f"colorbar: BLUE=low  ->  RED=high | saved {png_path}")
