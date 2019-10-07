import rospy 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig= plt.figure()
ax1 = plt.axes(xlim=(-10, 10), ylim=(-10, 10))

def initFigure():

	global fig
	global ax1

	ax1.set_xlabel('X')
	ax1.set_ylabel('Y')
	ax1.set_title('MPC Tracking Performance')
	ax1.grid()




def plotter(): 

	global ax1
	global fig

	rospy.init_node('plotter', anonymous=True)

	initFigure()
	plt.show()

	while not rospy.is_shutdown():
		a = 1


if __name__ == '__main__':

	try:
		plotter()
	except rospy.ROSInterruptException:
		pass