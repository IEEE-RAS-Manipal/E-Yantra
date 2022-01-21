red_count=0
blue_count=0

def truck_inventory(choice): #1 for red truck else blue truck
	global red_count,blue_count
	x=57.5
	y=63.75
	z=1.7
	redtruck = [
		  [[x,y,z],1],
		  [[x,y+1.23,z],2],
		  [[x,y+2.46,z],3],
		  [[x+0.85,y,z],4],
		  [[x+0.85,y+1.23,z],5],
		  [[x+0.85,y+2.46,z],6],
		  [[x+1.7,y,z],7],
		  [[x+1.7,y+1.23,z],8],
		  [[x+1.7,y+2.46,z],9],
		  [[x+2.55,y,z],10],
		  [[x+2.55,y+1.23,z],11],
		  [[x+2.55,y+2.46,z],12],
		]
	a=13.85
	b=-7.4
	c=1.7
	bluetruck = [
		  [[a,b,c],1],
		  [[a,b+1.23,c],2],
		  [[a,b+2.46,c],3],
		  [[a+0.85,b,c],4],
		  [[a+0.85,b+1.23,c],5],
		  [[a+0.85,b+2.46,c],6],
		  [[a+1.7,b,c],7],
		  [[a+1.7,b+1.23,c],8],
		  [[a+1.7,b+2.46,c],9],
		  [[a+2.55,b,c],10],
		  [[a+2.55,b+1.23,c],11],
		  [[a+2.55,b+2.46,c],12],
		]

	if(choice==1):
		red_count+=1
		for i in range(12):
			if redtruck[i][1]==red_count:
				return redtruck[red_count-1][0]
	else :
		blue_count+=1
		for i in range(12):
			if bluetruck[i][1]==blue_count:
				return bluetruck[blue_count-1][0]


print(truck_inventory(1))

