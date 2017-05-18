from math import sqrt

map_size = 150
res = 3
tower_size = 8
res_theta = 5

def distance(x0, y0, x1, y1):
	dsquare = (x1 - x0)**2 + (y1 - y0)**2
	return sqrt(dsquare)

class Localize:
	def __init__(self, x0, y0, theta0):
		self.grid_size = int(round(float(map_size) / res))
		self.bel, self.map = [], []

		for i in xrange(self.grid_size):
			self.bel.append([0] * self.grid_size)
			self.map.append([0] * self.grid_size)

		x0 = (x0 - 1) / res
		y0 = (y0 - 1) / res
		self.bel[x0][y0] = 1		

		mid = ((map_size / 2) - 1) / res
		bound = (tower_size / 2) / res
		for i in xrange(mid - bound, mid + bound + 1):
			for j in xrange(mid - bound, mid + bound + 1):
				self.map[i][j] = 1

		self.heading_grid_size = int(round(float(360) / res_theta))
		self.bel_heading = [0] * self.heading_grid_size
		self.bel_heading[theta0 / res_theta] = 1


	def updateHeading(self, dtheta, imu_data):
		dtheta = (dtheta - 1) / res_theta
		pdf = [(-2, 0.1), (-1, 0.2), (0, 0.4), (1, 0.2), (2, 0.1)]
		bel_heading = [0] * self.heading_grid_size
		for (i, b) in enumerate(self.bel_heading):
			if b > 0:
				for (j, p) in pdf:
					bel_heading[(i+j) % self.heading_grid_size] += p * b

		index = imu_data / res_theta
		self.bel_heading = [0] * self.heading_grid_size
		for (i, p) in pdf:
			self.bel_heading[(index+i) % self.heading_grid_size] += p * bel_heading[(index+i) % self.heading_grid_size]
		
		self.bel_heading = [b / sum(self.bel_heading) for b in self.bel_heading]
		print self.bel_heading

	def updatePosition(self, dx, dy):
		dx = (dx - 1) / res
		dy = (dy - 1) / res
		err = int(round(max(dx, dy) / float(4)))
		
		bel = []
		for i in xrange(self.grid_size):
			bel.append([0] * self.grid_size)

		for (x, row) in enumerate(self.bel):
			for (y, b) in enumerate(row):
				if b > 0:
					for i in xrange(max(0, x-err), min(self.grid_size, x+err+1)):
						for j in xrange(max(0, y-err), min(self.grid_size, y+err+1)):
							bel[i][j] += float(b)  / (err * err)

		self.bel = bel
		print self.bel
