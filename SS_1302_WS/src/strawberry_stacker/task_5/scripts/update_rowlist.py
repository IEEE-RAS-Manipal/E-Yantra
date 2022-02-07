rowlist =	{1:0,
			 2:0,
			 3:0,
			 4:0,
			 5:0,
			 6:0,
			 7:0,
			 8:0,
			 9:0,
			 10:0,
			 11:0,
			 12:0,
			 13:0,
			 14:0,
			 15:0
	
			}

def update_rowlist(rno):
	if rno in rowlist.keys():
		rowlist[rno]+=1
	print(rowlist)

update_rowlist(4)
update_rowlist(4)
