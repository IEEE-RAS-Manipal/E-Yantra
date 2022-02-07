truck=[[[56.5,62.75],[0,0]],
		   [[14.85,-8.4],[0,0]]]

def truck_inventory(n): #0 for red truck 1 blue truck
	global truck
	ri=truck[n][1][0]
	rj=truck[n][1][1]

	cell=[truck[n][0][0]+ri*0.85,truck[n][0][1]+rj*1.23,1.7]
	
	if(truck[n][1][1]==2):
		truck[n][1][1]=0
		truck[n][1][0]+=1
	else:
		truck[n][1][1]+=1
	print(ri)
	print(rj)
	print(cell)
	


