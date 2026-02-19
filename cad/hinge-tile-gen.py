#!/bin/python3

# Generates a grid of the rectangular 6mm delrin pieces

kerf = 0.05
nx   = 5
ny   = 2

ox = 10
oy = 10

w=12.00 + kerf*2
h=22.56 + kerf*2


def circle(x,y,r,stroke=""):
	s = f" stroke='{stroke}'" if stroke else ""
	print(f"<circle{s} cx=\"{x:.2f}\" cy=\"{y:.2f}\" r=\"{r}\"/>")

def polyline2(x1,y1,x2,y2,stroke=""):
	s = f" stroke='{stroke}'" if stroke else ""
	print(f"<polyline{s} points='{x1:.2f},{y1:.2f} {x2:.2f},{y2:.2f}'/>")

def polyline3(x1,y1,x2,y2,x3,y3,stroke=""):
	s = f" stroke='{stroke}'" if stroke else ""
	print(f"<polyline{s} points='{x1:.2f},{y1:.2f} {x2:.2f},{y2:.2f} {x3:.2f},{y3:.2f}'/>")

def polyline4(x1,y1,x2,y2,x3,y3,x4,y4,stroke=""):
	s = f" stroke='{stroke}'" if stroke else ""
	print(f"<polyline{s} points='{x1:.2f},{y1:.2f} {x2:.2f},{y2:.2f} {x3:.2f},{y3:.2f} {x4:.2f},{y4:.2f}'/>")

canvas_w = round(ox +nx*w +10)
canvas_h = round(oy +ny*h +10)

print(f"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {canvas_w} {canvas_h}\" width=\"{canvas_w}mm\" height=\"{canvas_h}mm\" fill=\"none\" stroke=\"#000\" stroke-width=\"0.2\">")
print(f"\n<!-- {nx}x{ny} kerf={kerf} -->")
print('<g>')


#colors=['#000','#f00','#0f0','#00f','#ff0','#0ff','#f0f','#fff']
colors=['#000','#f00','#0f0','#00f','#ff0','#0ff','#f0f','#fff']

# left
polyline2(ox-kerf,oy-kerf+ny*h, ox-kerf,oy-kerf, colors[0])


for x in range(nx):
	# circles
	for y in range(ny):
		circle( ox+6 +x*w, oy+5.28 +y*h,1.5 ,colors[x])
		circle( ox+6 +x*w, oy+16.56+y*h,1.5 ,colors[x])
	# horizontal dividing lines
	for y in range(1,ny):
		polyline2( ox-kerf +x*w,oy-kerf+y*h, ox-kerf+(x+1)*w,oy-kerf+y*h, colors[x])
	# outline
	polyline4( ox-kerf+x*w,oy-kerf, ox-kerf+(x+1)*w,oy-kerf, ox-kerf+(x+1)*w,oy-kerf+ny*h, ox-kerf+x*w,oy-kerf+ny*h,colors[x+1])
	print()

# vertical lines
#for x in range(1,nx):
#	polyline2(ox-kerf+x*w,oy-kerf, ox-kerf+x*w,oy-kerf+ny*h,'#0f0')

#print()

# horiz lines
#for y in range(1,ny):
#	polyline2(ox-kerf,oy-kerf+y*h, ox-kerf+nx*w,oy-kerf+y*h,'#0f0')

#print()

# bottom and right
#polyline3(ox-kerf+nx*w,oy-kerf, ox-kerf+nx*w,oy-kerf+ny*h, ox-kerf,oy-kerf+ny*h, '#00f')



print('</g>')
print()
