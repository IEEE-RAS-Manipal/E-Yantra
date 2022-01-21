truck=[[[56.5,62.75],[0,2]],
		   [[14.85,-8.4],[0,0]]]

def truck_inventory(choice): #0 for red truck 1 blue truck
	global truck
	ri=truck[choice][1][0]
	rj=truck[choice][1][1]

	cell=[truck[choice][0][0]+ri*0.85,truck[choice][0][1]+rj*1.23,1.7]
	rj+=1
	if(rj==2):
		rj=0
		ri+=1
	print(cell)
	truck[choice][1][0]=ri
	truck[choice][1][0]=rj
	


