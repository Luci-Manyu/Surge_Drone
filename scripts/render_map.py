#!/usr/bin/env python3
"""Render an RTAB-Map exported PLY cloud to a top-down PNG, no 3rd-party viz libs."""
import sys, struct, zlib, numpy as np

ply_path, png_path = sys.argv[1], sys.argv[2]

# --- parse binary_little_endian PLY vertex block ---
with open(ply_path, "rb") as f:
    raw = f.read()
hdr_end = raw.index(b"end_header\n") + len("end_header\n")
header = raw[:hdr_end].decode("ascii", "replace")
n = next(int(l.split()[2]) for l in header.splitlines() if l.startswith("element vertex"))
# property order: x y z (f4), r g b (u1), nx ny nz curvature (f4)  -> 31 bytes
dt = np.dtype([("x","<f4"),("y","<f4"),("z","<f4"),
               ("r","u1"),("g","u1"),("b","u1"),
               ("nx","<f4"),("ny","<f4"),("nz","<f4"),("c","<f4")])
v = np.frombuffer(raw[hdr_end:hdr_end + n*dt.itemsize], dtype=dt, count=n)
x, y, z = v["x"].astype(np.float64), v["y"].astype(np.float64), v["z"].astype(np.float64)
r, g, b = v["r"], v["g"], v["b"]

# --- top-down raster: bin into pixels, keep the highest-z point per cell ---
W = 1200
pad = 0.3
xmin, xmax, ymin, ymax = x.min()-pad, x.max()+pad, y.min()-pad, y.max()+pad
scale = (W-1) / max(xmax-xmin, ymax-ymin)
H = int((ymax-ymin)*scale) + 1
Wpx = int((xmax-xmin)*scale) + 1
img = np.full((H, Wpx, 3), 255, np.uint8)     # white background
zbuf = np.full((H, Wpx), -1e9, np.float64)

px = ((x - xmin)*scale).astype(np.int32)
py = ((ymax - y)*scale).astype(np.int32)       # flip so +y is up
order = np.argsort(z)                           # paint low->high so high z wins
for i in order:
    cx, cy = px[i], py[i]
    if z[i] > zbuf[cy, cx]:
        zbuf[cy, cx] = z[i]
        img[cy, cx] = (r[i], g[i], b[i])

# --- pure-stdlib PNG writer ---
def write_png(path, arr):
    h, w, _ = arr.shape
    rows = b"".join(b"\x00" + arr[i].tobytes() for i in range(h))
    def chunk(typ, data):
        c = struct.pack(">I", len(data)) + typ + data
        return c + struct.pack(">I", zlib.crc32(typ + data) & 0xffffffff)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(rows, 9)))
        f.write(chunk(b"IEND", b""))

write_png(png_path, img)
filled = int((zbuf > -1e8).sum())
print(f"points={n}  image={Wpx}x{H}px  filled_cells={filled}")
print(f"extent: X[{xmin:.1f},{xmax:.1f}] Y[{ymin:.1f},{ymax:.1f}] Z[{z.min():.1f},{z.max():.1f}] m")
print(f"saved {png_path}")
