from course import *
from Intersection import *

import scipy.stats as sp

i = Intersection()

c = i.courses["north", "left"]

#x,y,theta,speed = 3.25,-50,pi/2,10
x,y,theta = c.getPose(c.distance_to_crossing+c.radius*pi/4)
print x,y,theta
speed = 10

dev = 0.2
xd,yd,td,sd = [dev]*4

si = 15000

xs = np.random.normal(x, xd, si)
ys = np.random.normal(y, yd, si)
thetas = np.random.normal(theta, td, si)
speeds = np.random.normal(speed, sd, si)



r = np.array([c.getTimeToCrossing(*a, Is="go")for a in zip(xs,ys, thetas, speeds)])

m, s = np.mean(r), np.std(r)

print m,s

r = c.getTimeToCrossing(x,y,theta,speed, Is="go")
a = c.getTimeToCrossing(x+dev*0.8*cos(theta),y+dev*0.8*sin(theta),theta,speed-dev*0.8, Is="go")
b = c.getTimeToCrossing(x-dev*0.8*cos(theta),y-dev*0.8*sin(theta),theta,speed+dev*0.8, Is="go")



me = (a + b) / 2

print me
print a - me
print b - me

print 1 - sp.norm.cdf(0.2, 0, 0.2)
f =  (1 - sp.norm.pdf(0.17, 0, 0.2))
print f**2